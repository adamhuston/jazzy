"""PiSugar backup-UPS presence check.

The unit on this bot is a PiSugar S Plus: hardware-only, no I2C, no battery
telemetry. We can at most tell whether the optional ``pisugar-server`` is
running. Everything here is read-only and never raises.
"""

from __future__ import annotations

import socket
from dataclasses import dataclass

from .. import config


@dataclass(frozen=True)
class PiSugarStatus:
    present: bool
    model: str
    note: str


def _query(request: str, timeout: float = 0.4) -> str | None:
    """Ask pisugar-server one command over its UDS, then TCP. None if absent."""
    # Unix domain socket first (default install).
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.connect(config.PISUGAR_SOCK)
            sock.sendall(request.encode())
            return sock.recv(256).decode(errors="replace").strip()
    except Exception:
        pass
    try:
        with socket.create_connection(config.PISUGAR_TCP, timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(request.encode())
            return sock.recv(256).decode(errors="replace").strip()
    except Exception:
        return None


def detect() -> PiSugarStatus:
    """Return presence + model, without assuming any battery telemetry."""
    model = _query("get model")
    if model is None:
        return PiSugarStatus(False, "", "pisugar-server not reachable")
    name = model.split(":", 1)[-1].strip() if ":" in model else model
    return PiSugarStatus(True, name or "PiSugar", "UPS online (no battery telemetry)")
