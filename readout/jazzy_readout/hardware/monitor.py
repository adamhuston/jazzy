"""Background hardware poller.

Reading the serial port and querying pisugar-server both block, so a dedicated
thread samples them on a slow cadence and publishes an immutable snapshot the
UI can read without stalling.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from .. import config
from .accessories import Accessory, enumerate_accessories
from .pisugar import PiSugarStatus, detect as detect_pisugar
from .yahboom import YahboomBattery, YahboomError


@dataclass(frozen=True)
class HardwareSnapshot:
    battery_volts: float | None
    battery_error: str | None
    pisugar: PiSugarStatus | None
    accessories: tuple[Accessory, ...]


class HardwareMonitor:
    """Polls battery/pisugar/accessories on a background thread."""

    def __init__(self, port: str | None = None, enable_battery: bool = True):
        self._port = port if port is not None else config.BATTERY_PORT
        self._enable_battery = enable_battery
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._battery = YahboomBattery(self._port) if enable_battery else None
        self._battery_open = False
        self._snapshot = HardwareSnapshot(None, None if enable_battery else "disabled", None, ())

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="jazzwatch-hw", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._battery is not None:
            self._battery.close()

    def snapshot(self) -> HardwareSnapshot:
        with self._lock:
            return self._snapshot

    def _run(self) -> None:
        while not self._stop.is_set():
            volts, err = self._read_battery()
            pisugar = detect_pisugar()
            accessories = tuple(enumerate_accessories())
            with self._lock:
                self._snapshot = HardwareSnapshot(volts, err, pisugar, accessories)
            self._stop.wait(config.BATTERY_POLL_S)

    def _read_battery(self) -> tuple[float | None, str | None]:
        if self._battery is None:
            return None, "disabled"
        try:
            if not self._battery_open:
                self._battery.open()
                self._battery.enable_auto_report()
                self._battery_open = True
            self._battery.flush()
            volts = self._battery.read_voltage(timeout=1.5)
            if volts is None:
                return None, "no battery frame"
            return volts, None
        except YahboomError as exc:
            self._battery_open = False
            return None, str(exc)
