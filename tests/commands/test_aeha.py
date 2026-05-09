"""Tests for the AEHA IR command encoder."""

import pytest

from infrared_protocols.commands.aeha import AEHACommand

# Panasonic TU-MHD500 sample.
CUSTOMER_CODE = 0x2002
COMMAND = 0x028D0F80
COMMAND_BITS = 32

COMMON_PREFIX: list[int] = [
    #region Leader (8T high, 4T low)
    3400,
    -1700,
    #endregion Leader (8T high, 4T low)
    #region Customer code (16 bits, LSB first)
    425,
    -425,
    425,
    -1275,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -1275,
    425,
    -425,
    425,
    -425,
    #endregion Customer code (16 bits, LSB first)
    #region Command (32 bits, LSB first)
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -1275,
    425,
    -1275,
    425,
    -1275,
    425,
    -1275,
    425,
    -1275,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -1275,
    425,
    -425,
    425,
    -1275,
    425,
    -1275,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -1275,
    425,
    -425,
    425,
    -1275,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    425,
    -425,
    #endregion Command (32 bits, LSB first)
]

FRAME_WITHOUT_REPEAT: list[int] = [
    *COMMON_PREFIX,
    #region Trailer (1T high, 8ms low)
    425,
    -8000,
    #endregion Trailer (1T high, 8ms low)
]

FRAME_WITH_TWO_REPEATS: list[int] = [
    *COMMON_PREFIX,
    #region Trailer (1T high, 130ms-frame low)
    425,
    -73475,
    #endregion Trailer (1T high, 130ms-frame low)
    #region Repeat (8T high, 8T low)
    3400,
    -3400,
    #endregion Repeat (8T high, 8T low)
    #region Trailer (1T high, 130ms-frame low)
    425,
    -123200,
    #endregion Trailer (1T high, 130ms-frame low)
    #region Repeat (8T high, 8T low)
    3400,
    -3400,
    #endregion Repeat (8T high, 8T low)
    #region Trailer (1T high, 8ms low)
    425,
    -8000,
    #endregion Trailer (1T high, 8ms low)
]


def test_aeha_command_get_raw_timings_no_repeat() -> None:
    """Test AEHA raw timings for the Panasonic TU-MHD500 sample code."""
    command = AEHACommand(
        customer_code=CUSTOMER_CODE,
        command=COMMAND,
        command_bits=COMMAND_BITS,
        include_parity=False,
    )
    assert command.get_raw_timings() == FRAME_WITHOUT_REPEAT
    assert command.modulation == 38000


def test_aeha_command_get_raw_timings_with_repeats() -> None:
    """Test AEHA raw timings with two repeat codes."""
    command = AEHACommand(
        customer_code=CUSTOMER_CODE,
        command=COMMAND,
        command_bits=COMMAND_BITS,
        include_parity=False,
        repeat_count=2,
    )
    assert command.get_raw_timings() == FRAME_WITH_TWO_REPEATS


def test_aeha_command_get_raw_timings_with_customer_parity() -> None:
    """Test optional customer-code parity insertion."""
    without_parity = AEHACommand(
        customer_code=CUSTOMER_CODE,
        command=COMMAND,
        command_bits=COMMAND_BITS,
        include_parity=False,
    )
    with_parity = AEHACommand(
        customer_code=CUSTOMER_CODE,
        command=COMMAND,
        command_bits=COMMAND_BITS,
        include_parity=True,
    )

    without_parity_timings = without_parity.get_raw_timings()
    with_parity_timings = with_parity.get_raw_timings()

    assert with_parity_timings != without_parity_timings
    assert len(with_parity_timings) == len(without_parity_timings) + 8

    decoded = AEHACommand.from_raw_timings(with_parity_timings)
    assert decoded is not None
    assert decoded.include_parity is True
    assert decoded.command == COMMAND
    assert decoded.command_bits == COMMAND_BITS


def test_aeha_command_validation() -> None:
    """Test AEHA input validation errors."""
    with pytest.raises(ValueError, match="customer_code"):
        AEHACommand(customer_code=0x1_0000, command=0)

    with pytest.raises(ValueError, match="command_bits"):
        AEHACommand(customer_code=0x0000, command=0, command_bits=0)

    with pytest.raises(ValueError, match="command does not fit"):
        AEHACommand(customer_code=0x0000, command=0x100, command_bits=8)


def test_aeha_command_from_raw_timings_no_repeat() -> None:
    """Test decoding the Panasonic TU-MHD500 sample frame."""
    decoded = AEHACommand.from_raw_timings(FRAME_WITHOUT_REPEAT)

    assert decoded is not None
    assert decoded.customer_code == CUSTOMER_CODE
    assert decoded.command == COMMAND
    assert decoded.command_bits == COMMAND_BITS
    assert decoded.include_parity is False
    assert decoded.repeat_count == 0
    assert decoded.modulation == 38000


def test_aeha_command_from_raw_timings_with_repeats() -> None:
    """Test decoding Panasonic TU-MHD500 sample frame with repeats."""
    decoded = AEHACommand.from_raw_timings(FRAME_WITH_TWO_REPEATS)

    assert decoded is not None
    assert decoded.customer_code == CUSTOMER_CODE
    assert decoded.command == COMMAND
    assert decoded.command_bits == COMMAND_BITS
    assert decoded.include_parity is False
    assert decoded.repeat_count == 2


def test_aeha_command_from_raw_timings_within_tolerance() -> None:
    """Test decoding succeeds when timings deviate within tolerance."""
    skewed = [
        int(t * (500 / 425)) if t > 0 else int(t * (350 / 425))
        for t in FRAME_WITHOUT_REPEAT[:-1]
    ]
    skewed.append(FRAME_WITHOUT_REPEAT[-1])

    decoded = AEHACommand.from_raw_timings(skewed)

    assert decoded is not None
    assert decoded.customer_code == CUSTOMER_CODE
    assert decoded.command == COMMAND
    assert decoded.command_bits == COMMAND_BITS
    assert decoded.include_parity is False


def test_aeha_command_from_raw_timings_invalid_customer_parity() -> None:
    """Test decoding rejects a frame with an invalid customer-code parity nibble."""
    timings = AEHACommand(
        customer_code=CUSTOMER_CODE,
        command=COMMAND,
        command_bits=COMMAND_BITS,
        include_parity=True,
    ).get_raw_timings()

    invalid_timings = [*timings[:34], -425, *timings[35:]]
    assert AEHACommand.from_raw_timings(invalid_timings) is None


@pytest.mark.parametrize(
    "timings",
    [
        pytest.param([], id="empty"),
        pytest.param([3400, -1700, 425], id="too_short"),
        pytest.param(
            [1000, -1700, *([425, -425] * 21), 425],
            id="invalid_leader_high",
        ),
        pytest.param(
            [3400, -100, *([425, -425] * 21), 425],
            id="invalid_leader_low",
        ),
        pytest.param(
            [*FRAME_WITHOUT_REPEAT[:3], -3000, *FRAME_WITHOUT_REPEAT[4:]],
            id="invalid_bit",
        ),
        pytest.param([*FRAME_WITHOUT_REPEAT, -12345], id="trailing_garbage"),
        pytest.param(
            [*FRAME_WITHOUT_REPEAT[:-1], 3400, -3400, 425],
            id="incomplete_repeat",
        ),
    ],
)
def test_aeha_command_from_raw_timings_invalid(timings: list[int]) -> None:
    """Test decoder returns None for malformed inputs."""
    assert AEHACommand.from_raw_timings(timings) is None
