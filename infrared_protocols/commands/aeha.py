"""AEHA (Association for Electric Home Appliances, Japan) IR command."""

from typing import Self, override

from . import Command

T = 425
MIN_T = 350
MAX_T = 500
LEADER_HIGH = 8 * T
LEADER_LOW = 4 * T
BIT_HIGH = T
ZERO_LOW = T
ONE_LOW = 3 * T
REPEAT_HIGH = 8 * T
REPEAT_LOW = 8 * T
TRAILER_LOW = 8000
FRAME_GAP = 130000


class AEHACommand(Command):
    """AEHA IR command."""

    customer_code: int
    command: int
    command_bits: int
    include_parity: bool

    def __init__(
        self,
        *,
        customer_code: int,
        command: int,
        command_bits: int = 48,
        include_parity: bool = True,
        modulation: int = 38000,
        repeat_count: int = 0,
    ) -> None:
        """Initialize the AEHA IR command."""
        if not 0 <= customer_code <= 0xFFFF:
            raise ValueError("AEHA customer_code must be in range 0x0000..0xFFFF")
        if command_bits <= 0:
            raise ValueError("AEHA command_bits must be a positive integer")
        if not 0 <= command < (1 << command_bits):
            raise ValueError("AEHA command does not fit in command_bits")
        super().__init__(modulation=modulation, repeat_count=repeat_count)
        self.customer_code = customer_code
        self.command = command
        self.command_bits = command_bits
        self.include_parity = include_parity

    @staticmethod
    def _get_customer_code_parity(customer_code: int) -> int:
        """Return the 4-bit XOR parity for ``customer_code``."""
        parity = 0
        for i in range(4):
            parity ^= (customer_code >> (i * 4)) & 0xF
        return parity

    @staticmethod
    def _append_lsb_bits(timings: list[int], value: int, bit_count: int) -> None:
        """Append ``bit_count`` bits of ``value`` in LSB-first order."""
        for i in range(bit_count):
            bit = (value >> i) & 1
            timings.append(BIT_HIGH)
            timings.append(-ONE_LOW if bit else -ZERO_LOW)

    @staticmethod
    def _append_signed_us(timings: list[int], value: int) -> None:
        """Append a signed duration, merging with the last entry if possible."""
        if timings and (timings[-1] > 0) == (value > 0):
            timings[-1] += value
        else:
            timings.append(value)

    @override
    def get_raw_timings(self) -> list[int]:
        """Get raw timings for the AEHA command.

        AEHA protocol timing:
        - T (Timing unit): 350-500µs (425µs typ.)
        - Leader pulse: 8T high, 4T low
        - Logical '0': 1T high, 1T low
        - Logical '1': 1T high, 3T low
        - Trailer: 1T high, >=8ms low
        - Repeat code: 8T high, 8T low
        - Frame gap (Frame start to Repeat code): 130ms typ.

        Data format (LSB first):
        - customer code (16-bit)
        - parity (4-bit, XOR of customer-code nibbles)
        - data (variable length, 48-bit typ.)
        """
        frame: list[int] = [LEADER_HIGH, -LEADER_LOW]

        # Customer code (16-bit, LSB first)
        self._append_lsb_bits(frame, self.customer_code, 16)

        if self.include_parity:
            parity = self._get_customer_code_parity(self.customer_code)
            self._append_lsb_bits(frame, parity, 4)

        # Protocol payload (variable length, 48 bits typical)
        self._append_lsb_bits(frame, self.command, self.command_bits)

        # Trailer
        frame.append(BIT_HIGH)

        frame_duration = sum(abs(timing) for timing in frame)
        off_duration = max(FRAME_GAP - frame_duration, TRAILER_LOW)
        repeat_off_duration = max(FRAME_GAP - REPEAT_HIGH - REPEAT_LOW, TRAILER_LOW)

        timings: list[int] = list(frame)

        if self.repeat_count == 0:
            timings.append(-TRAILER_LOW)
            return timings

        timings.append(-off_duration)
        for repeat_idx in range(self.repeat_count):
            is_last = repeat_idx == self.repeat_count - 1
            timings.extend([REPEAT_HIGH, -REPEAT_LOW, BIT_HIGH])
            timings.append(-TRAILER_LOW if is_last else -repeat_off_duration)

        return timings

    @classmethod
    def from_raw_timings(cls, timings: list[int]) -> Self | None:
        """Decode raw IR timings into an AEHACommand.

        Returns an AEHACommand if the timings match, or None otherwise.
        """
        # Minimum without parity: leader pair (2) + customer bits (16*2)
        # + one data bit (2) + trailer pair (2).
        if len(timings) < 38:
            return None

        frame_end = cls._decode_frame(timings, 0)
        if frame_end is None:
            return None

        bits = frame_end[0]
        next_index = frame_end[1]

        # Need 16-bit customer plus at least 1 following bit.
        if len(bits) < 17:
            return None

        customer_code = 0
        for bit_index in range(16):
            customer_code |= bits[bit_index] << bit_index

        candidates: list[tuple[bool, int]] = [(False, 16)]
        parity = cls._get_customer_code_parity(customer_code)
        if len(bits) >= 20:
            parity_from_frame = 0
            for bit_index in range(4):
                parity_from_frame |= bits[16 + bit_index] << bit_index
            if parity_from_frame == parity:
                candidates.append((True, 20))

        byte_aligned_candidates = [
            candidate for candidate in candidates if (len(bits) - candidate[1]) % 8 == 0
        ]
        selected_candidate = (
            byte_aligned_candidates[0]
            if len(byte_aligned_candidates) == 1
            else candidates[-1]
        )
        include_parity, command_start = selected_candidate

        command_bits = len(bits) - command_start
        command = 0
        for bit_index in range(command_bits):
            command |= bits[command_start + bit_index] << bit_index

        if (
            next_index >= len(timings)
            or timings[next_index] >= 0
            or -timings[next_index] < TRAILER_LOW
        ):
            return None

        repeat_count = 0
        i = next_index + 1
        while i < len(timings):
            # Each repeat block: REPEAT_HIGH, -REPEAT_LOW, BIT_HIGH, -OFF
            if i + 3 >= len(timings):
                return None

            if not cls._is_close(timings[i], REPEAT_HIGH):
                return None
            if not cls._is_close(-timings[i + 1], REPEAT_LOW):
                return None
            if not cls._is_close(timings[i + 2], BIT_HIGH):
                return None
            if timings[i + 3] >= 0 or -timings[i + 3] < TRAILER_LOW:
                return None

            repeat_count += 1
            i += 4

        return cls(
            customer_code=customer_code,
            command=command,
            command_bits=command_bits,
            include_parity=include_parity,
            repeat_count=repeat_count,
        )

    @staticmethod
    def _is_close(actual: int, expected: int) -> bool:
        """Check if an actual timing value is within tolerance of expected."""
        lower = (expected * MIN_T) // T
        upper = (expected * MAX_T) // T
        return lower <= actual <= upper

    @staticmethod
    def _decode_bit(high_us: int, low_us: int) -> int | None:
        """Decode a single AEHA data bit from high and low timings."""
        if not AEHACommand._is_close(high_us, BIT_HIGH):
            return None
        if AEHACommand._is_close(low_us, ZERO_LOW):
            return 0
        if AEHACommand._is_close(low_us, ONE_LOW):
            return 1
        return None

    @staticmethod
    def _decode_frame(
        timings: list[int], start_index: int
    ) -> tuple[list[int], int] | None:
        """Decode one AEHA frame starting at ``start_index``.

        Returns the decoded data bits and the index of the following off interval.
        """
        if start_index + 1 >= len(timings):
            return None
        if not AEHACommand._is_close(
            timings[start_index], LEADER_HIGH
        ) or not AEHACommand._is_close(-timings[start_index + 1], LEADER_LOW):
            return None

        bits: list[int] = []
        i = start_index + 2
        while i < len(timings):
            if i == len(timings) - 1:
                if AEHACommand._is_close(timings[i], BIT_HIGH):
                    return bits, i + 1
                return None

            bit = AEHACommand._decode_bit(timings[i], -timings[i + 1])
            if bit is None:
                if AEHACommand._is_close(timings[i], BIT_HIGH):
                    return bits, i + 1
                return None

            bits.append(bit)
            i += 2

        return None
