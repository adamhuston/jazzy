#!/usr/bin/env python3
"""Self-contained battery-voltage reader for the Yahboom ROS Expansion Board V3.0.

The board (STM32F103RCT6) auto-reports telemetry over its CH340 USB-serial link
at 115200 baud. This module speaks just enough of the Rosmaster serial protocol
to pull the pack voltage out of the "report speed" frame — no vendor library,
no Google-Drive download. Only `pyserial` is required.

Protocol (device -> host frame):

    0xFF 0xFB LEN FUNC D0 D1 ... Dn CHK

    HEAD = 0xFF
    RECV_ID = 0xFB              (device -> host; host -> device uses 0xFC)
    LEN  = FUNC + all data + CHK (so data length = LEN - 2, i.e. LEN - 3 after
           also dropping the leading id byte from our two-byte header sync)
    FUNC = 0x0A (FUNC_REPORT_SPEED) carries: vx,vy,vz (int16 LE, mm/s) + battery
    CHK  = (LEN + FUNC + sum(data)) & 0xFF

For FUNC_REPORT_SPEED the 7th data byte (index 6) is the pack voltage in
decivolts, so volts = data[6] / 10.0.

Run it directly on the Pi to sanity-check the reading:

    pip install pyserial
    python3 jazzy_readout/hardware/yahboom.py            # one reading
    python3 jazzy_readout/hardware/yahboom.py --watch    # live readings
    python3 jazzy_readout/hardware/yahboom.py --debug    # dump raw frames

A healthy pack should land in the ~7-13 V range. If nothing prints, use
--debug to see which FUNC codes the board is actually emitting.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections import Counter
from dataclasses import dataclass

try:
    import serial  # pyserial
except ImportError:  # keep the module importable without the dep (mock/SSH runs)
    serial = None  # type: ignore[assignment]

HEAD = 0xFF
RECV_ID = 0xFB
SEND_ID = 0xFC
COMPLEMENT = 257 - SEND_ID  # = 5, used only in host -> device checksums
FUNC_AUTO_REPORT = 0x01
FUNC_REPORT_SPEED = 0x0A

BAUD = 115200
DEFAULT_PORT = "/dev/myserial"  # udev-stable name from the board's myserial.rules
FALLBACK_PORTS = ("/dev/ttyUSB0", "/dev/ttyACM0")
SANE_VOLTAGE = (2.0, 30.0)  # reject obvious garbage bytes


class YahboomError(RuntimeError):
    """Raised when the serial port cannot be opened or read."""


@dataclass
class Frame:
    ext_len: int
    func: int
    data: bytes
    chk: int
    ok: bool  # True when the frame checksum matched


def pick_port(explicit: str | None = None) -> str:
    """Return the first plausible serial port for the board."""
    if explicit:
        return explicit
    for candidate in (DEFAULT_PORT, *FALLBACK_PORTS):
        if os.path.exists(candidate):
            return candidate
    return DEFAULT_PORT


class YahboomBattery:
    """Minimal reader for the board's auto-reported battery voltage."""

    def __init__(self, port: str | None = None, baud: int = BAUD, timeout: float = 1.0):
        self.port = pick_port(port)
        self.baud = baud
        self.timeout = timeout
        self._ser = None

    def open(self) -> "YahboomBattery":
        if serial is None:
            raise YahboomError("pyserial is not installed (pip install pyserial)")
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except Exception as exc:  # missing port / permission denied
            raise YahboomError(f"cannot open {self.port}: {exc}") from exc
        return self

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self) -> "YahboomBattery":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    def enable_auto_report(self, forever: bool = False) -> None:
        """Best-effort nudge in case the board is not auto-reporting already."""
        if self._ser is None:
            return
        state1 = 0x01
        state2 = 0x5F if forever else 0x00
        cmd = [HEAD, SEND_ID, 0x05, FUNC_AUTO_REPORT, state1, state2]
        cmd.append((sum(cmd, COMPLEMENT)) & 0xFF)
        try:
            self._ser.write(bytearray(cmd))
        except Exception:
            pass

    def _read_frame(self) -> Frame | None:
        """Sync on the header and return one decoded frame (or None on timeout)."""
        ser = self._ser
        if ser is None:
            raise YahboomError("port is not open")
        first = ser.read(1)
        if not first or first[0] != HEAD:
            return None
        second = ser.read(1)
        if not second or second[0] != RECV_ID:
            return None
        hdr = ser.read(2)  # LEN, FUNC
        if len(hdr) < 2:
            return None
        ext_len, func = hdr[0], hdr[1]
        ndata = ext_len - 3  # LEN counts FUNC + data + CHK; strip FUNC, CHK, and the id
        if ndata < 0:
            return None
        data = ser.read(ndata)
        if len(data) < ndata:
            return None
        chk = ser.read(1)
        if not chk:
            return None
        calc = (ext_len + func + sum(data)) & 0xFF
        return Frame(
            ext_len=ext_len,
            func=func,
            data=bytes(data),
            chk=chk[0],
            ok=(calc == chk[0]),
        )

    def read_voltage(self, timeout: float = 2.0) -> float | None:
        """Return pack voltage in volts, or None if no valid frame arrives in time."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._read_frame()
            if frame and frame.func == FUNC_REPORT_SPEED and len(frame.data) >= 7:
                volts = frame.data[6] / 10.0
                if SANE_VOLTAGE[0] <= volts <= SANE_VOLTAGE[1]:
                    return volts
        return None

    def dump_frames(self, seconds: float = 5.0) -> None:
        """Print every frame seen — used to verify framing on real hardware."""
        deadline = time.monotonic() + seconds
        seen: Counter[int] = Counter()
        while time.monotonic() < deadline:
            frame = self._read_frame()
            if not frame:
                continue
            seen[frame.func] += 1
            calc = (frame.ext_len + frame.func + sum(frame.data)) & 0xFF
            print(
                f"func=0x{frame.func:02X} len={frame.ext_len:2d}"
                f" data={frame.data.hex(' ')}"
                f" chk=0x{frame.chk:02X} calc=0x{calc:02X}"
            )
        print("\nFUNC codes seen:", {f"0x{k:02X}": v for k, v in seen.items()})

    def dump_raw(self, seconds: float = 3.0, chunk: int = 64) -> None:
        """Print the raw byte stream so framing/checksum can be reverse-engineered."""
        ser = self._ser
        if ser is None:
            raise YahboomError("port is not open")
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            buf = ser.read(chunk)
            if buf:
                print(buf.hex(" "))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Read Yahboom board battery voltage.")
    parser.add_argument("--port", default=None, help="serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--timeout", type=float, default=2.0, help="read timeout (s)")
    parser.add_argument("--watch", action="store_true", help="print readings forever")
    parser.add_argument("--debug", action="store_true", help="dump raw frames")
    parser.add_argument("--raw", action="store_true", help="dump raw byte stream")
    parser.add_argument("--seconds", type=float, default=5.0, help="--debug duration")
    args = parser.parse_args(argv)

    if serial is None:
        print("pyserial is not installed. Run: pip install pyserial", file=sys.stderr)
        return 2

    try:
        with YahboomBattery(args.port, args.baud) as bat:
            bat.enable_auto_report()
            print(f"port: {bat.port} @ {bat.baud} baud", file=sys.stderr)
            if args.raw:
                bat.dump_raw(args.seconds)
                return 0
            if args.debug:
                bat.dump_frames(args.seconds)
                return 0
            if args.watch:
                while True:
                    volts = bat.read_voltage(args.timeout)
                    print("no data" if volts is None else f"{volts:.1f} V")
            volts = bat.read_voltage(args.timeout)
            if volts is None:
                print("no valid battery frame (try --debug)", file=sys.stderr)
                return 1
            print(f"{volts:.1f} V")
            return 0
    except YahboomError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
