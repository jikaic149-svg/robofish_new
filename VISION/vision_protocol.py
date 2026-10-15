"""Shared PC-side helpers for fixed-period vision UDP commands.

The wire steering convention is positive=left, matching the path-guidance
control law.  The ESP32 translates that sign at the UDP boundary before
reusing its legacy HTTP actuator mixer.
"""

from __future__ import annotations

import struct


VISION_UDP_MAGIC = 0xA5
VISION_UDP_VERSION = 1
VISION_UDP_TYPE_COMMAND = 1
VISION_UDP_FRAME = struct.Struct("<BBBBIIBbBBH")


def _clamp_int(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, int(value)))


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= int(byte) << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def pack_vision_udp_frame(
    sequence: int,
    timestamp_ms: int,
    power_percent: int,
    steering_percent: int,
    brake: bool,
) -> bytes:
    frame = VISION_UDP_FRAME.pack(
        VISION_UDP_MAGIC,
        VISION_UDP_VERSION,
        VISION_UDP_TYPE_COMMAND,
        12,
        int(sequence) & 0xFFFFFFFF,
        int(timestamp_ms) & 0xFFFFFFFF,
        _clamp_int(power_percent, 0, 100),
        _clamp_int(steering_percent, -100, 100),
        1 if brake else 0,
        0,
        0,
    )
    return frame[:-2] + struct.pack("<H", crc16_ccitt(frame[:-2]))
