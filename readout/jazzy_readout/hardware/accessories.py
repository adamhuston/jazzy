"""Enumerate USB-serial accessories (Yahboom board, RPLidar, ...).

Uses pyserial's ``list_ports`` when available so it works the same on the Pi
and the dev box. Read-only; degrades gracefully if pyserial is missing.
"""

from __future__ import annotations

from dataclasses import dataclass

try:
    from serial.tools import list_ports
except Exception:  # pyserial not installed
    list_ports = None  # type: ignore[assignment]

# VID:PID (lowercase) -> friendly role.
USB_HINTS = {
    "10c4:ea60": "RPLidar (CP210x UART)",
    "1a86:7523": "Yahboom board (CH340 UART)",
    "1a86:55d4": "Yahboom board (CH9102 UART)",
    "0403:6001": "FTDI FT232 UART",
    "0483:5740": "STM32 Virtual COM",
}


@dataclass(frozen=True)
class Accessory:
    device: str
    vid_pid: str
    label: str


def enumerate_accessories() -> list[Accessory]:
    """List attached serial devices, annotated with a friendly role."""
    if list_ports is None:
        return []
    out: list[Accessory] = []
    for port in sorted(list_ports.comports(), key=lambda p: p.device):
        if port.vid is not None and port.pid is not None:
            key = f"{port.vid:04x}:{port.pid:04x}"
        else:
            key = ""
        label = USB_HINTS.get(key) or (port.description or "serial device")
        out.append(Accessory(port.device, key or "----:----", label))
    return out
