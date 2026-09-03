"""Bridge between the ROS 2 graph and the jazzwatch UI.

A background thread owns all ROS I/O (or synthesises data in ``--mock`` mode)
and publishes an immutable snapshot the Textual app polls on its refresh tick.
The UI never touches rclpy directly, so it stays responsive and testable off
the robot.
"""

from __future__ import annotations

import random
import threading
import time
from collections import deque
from dataclasses import dataclass

from . import config

try:  # ROS is only present on the robot; the dev box falls back to mock.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    from rov2_interfaces.msg import SystemStatus

    RCLPY_AVAILABLE = True
except Exception:  # noqa: BLE001 - any import failure means "no ROS here"
    RCLPY_AVAILABLE = False

SYSTEM_STATES = {
    0: "INIT",
    1: "READY",
    2: "ACTIVE",
    3: "DEGRADED",
    4: "FAULT",
    5: "SHUTDOWN",
}

PLUGIN_STATES = {
    0: "UNKNOWN",
    1: "UNCONFIGURED",
    2: "INACTIVE",
    3: "ACTIVE",
    4: "DEGRADED",
    5: "FAULT",
}


@dataclass(frozen=True)
class PluginView:
    name: str
    type: str
    category: str
    state: int
    message: str

    @property
    def state_label(self) -> str:
        return PLUGIN_STATES.get(self.state, str(self.state))


@dataclass(frozen=True)
class SystemView:
    state: int
    state_label: str
    loop_count: int
    loop_period_ms: float
    loop_jitter_ms: float
    plugins: tuple[PluginView, ...] = ()


@dataclass(frozen=True)
class DiagView:
    level: int
    name: str
    message: str


@dataclass(frozen=True)
class LogEvent:
    ts: float
    level: str  # info | good | warn | bad
    text: str


@dataclass(frozen=True)
class Snapshot:
    online: bool
    age_s: float
    source: str
    system: SystemView | None
    diagnostics: tuple[DiagView, ...] = ()
    log: tuple[LogEvent, ...] = ()
    log_seq: int = 0


class RosBridge:
    """Owns the ROS thread and exposes thread-safe snapshots."""

    def __init__(self, mock: bool = False):
        self.mock = mock or not RCLPY_AVAILABLE
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._system: SystemView | None = None
        self._diagnostics: tuple[DiagView, ...] = ()
        self._last_status: float = 0.0
        self._log: deque[LogEvent] = deque(maxlen=config.RUNLOG_MAX_LINES)
        self._seq = 0
        self._was_online = False

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        target = self._run_mock if self.mock else self._run_ros
        self._thread = threading.Thread(target=target, name="jazzwatch-ros", daemon=True)
        self._thread.start()
        self._log_event("info", "mock data source" if self.mock else "ROS bridge started")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # -- public read --------------------------------------------------------
    def snapshot(self) -> Snapshot:
        now = time.monotonic()
        with self._lock:
            age = now - self._last_status if self._last_status else float("inf")
            online = age <= config.OFFLINE_TIMEOUT_S
            if online != self._was_online:
                self._was_online = online
                self._append_locked(
                    "good" if online else "bad",
                    "framework online" if online else "framework OFFLINE",
                )
            return Snapshot(
                online=online,
                age_s=age,
                source="mock" if self.mock else "ros",
                system=self._system,
                diagnostics=self._diagnostics,
                log=tuple(self._log),
                log_seq=self._seq,
            )

    # -- ingest (shared by ros + mock) -------------------------------------
    def _ingest_status(self, view: SystemView) -> None:
        with self._lock:
            prev = self._system
            self._diff_and_log(prev, view)
            self._system = view
            self._last_status = time.monotonic()

    def _ingest_diagnostics(self, diags: tuple[DiagView, ...]) -> None:
        with self._lock:
            self._diagnostics = diags

    def _diff_and_log(self, prev: SystemView | None, cur: SystemView) -> None:
        if prev is None or prev.state != cur.state:
            self._append_locked(_state_level(cur.state), f"system -> {cur.state_label}")
        prev_plugins = {p.name: p for p in (prev.plugins if prev else ())}
        cur_plugins = {p.name: p for p in cur.plugins}
        for name, p in cur_plugins.items():
            old = prev_plugins.get(name)
            if old is None:
                self._append_locked("info", f"+ {p.category} '{name}' [{p.state_label}]")
            elif old.state != p.state:
                self._append_locked(
                    _plugin_level(p.state),
                    f"'{name}' {old.state_label} -> {p.state_label}"
                    + (f": {p.message}" if p.message else ""),
                )
        for name in prev_plugins.keys() - cur_plugins.keys():
            self._append_locked("warn", f"- '{name}' removed")

    def _append_locked(self, level: str, text: str) -> None:
        """Append a log event. Caller must already hold ``self._lock``."""
        self._log.append(LogEvent(time.time(), level, text))
        self._seq += 1

    def _log_event(self, level: str, text: str) -> None:
        with self._lock:
            self._append_locked(level, text)

    # -- ROS feeder ---------------------------------------------------------
    def _run_ros(self) -> None:
        rclpy.init(args=None)
        node = rclpy.create_node("jazzwatch")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        node.create_subscription(SystemStatus, config.STATUS_TOPIC, self._on_status, qos)
        node.create_subscription(
            DiagnosticArray, config.DIAGNOSTICS_TOPIC, self._on_diag, qos
        )
        try:
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def _on_status(self, msg) -> None:
        plugins = tuple(
            PluginView(p.name, p.type, p.category, p.state, p.message) for p in msg.plugins
        )
        self._ingest_status(
            SystemView(
                state=msg.state,
                state_label=msg.state_label or SYSTEM_STATES.get(msg.state, str(msg.state)),
                loop_count=msg.loop_count,
                loop_period_ms=msg.loop_period_ms,
                loop_jitter_ms=msg.loop_jitter_ms,
                plugins=plugins,
            )
        )

    def _on_diag(self, msg) -> None:
        diags = tuple(DiagView(s.level, s.name, s.message) for s in msg.status)
        self._ingest_diagnostics(diags)

    # -- Mock feeder --------------------------------------------------------
    def _run_mock(self) -> None:
        scenario = _MockScenario()
        while not self._stop.is_set():
            view, diags = scenario.step()
            self._ingest_status(view)
            self._ingest_diagnostics(diags)
            time.sleep(0.5)


def _state_level(state: int) -> str:
    return {2: "good", 1: "info", 3: "warn", 4: "bad", 5: "warn"}.get(state, "info")


def _plugin_level(state: int) -> str:
    return {3: "good", 2: "info", 4: "warn", 5: "bad"}.get(state, "info")


class _MockScenario:
    """Synthesises a plausible boot-to-active sequence for off-robot demos."""

    def __init__(self):
        self._t = 0
        self._plugins = [
            ["imu", "rov2/ImuPlugin", "sensor", 2],
            ["yahboom_driver", "rov2/MotorDriver", "actuator", 2],
            ["lidar", "rov2/RplidarPlugin", "sensor", 1],
            ["pilot", "rov2/PilotBrain", "brain", 1],
        ]

    def step(self) -> tuple[SystemView, tuple[DiagView, ...]]:
        self._t += 1
        # Ramp system state INIT -> READY -> ACTIVE over the first few ticks.
        state = 0 if self._t < 2 else 1 if self._t < 5 else 2
        # Plugins come alive as the system activates.
        if self._t == 4:
            self._plugins[2][3] = 3  # lidar active
            self._plugins[3][3] = 3  # pilot active
        # Occasional degrade to exercise the warning colours.
        if self._t % 23 == 0:
            self._plugins[0][3] = 4
        elif self._t % 23 == 3:
            self._plugins[0][3] = 3
        plugins = tuple(
            PluginView(n, t, c, s, "" if s in (2, 3) else "waiting") for n, t, c, s in self._plugins
        )
        view = SystemView(
            state=state,
            state_label=SYSTEM_STATES[state],
            loop_count=self._t * 20,
            loop_period_ms=50.0,
            loop_jitter_ms=round(random.uniform(0.2, 2.5), 2),
            plugins=plugins,
        )
        diags = tuple(DiagView(0 if p.state == 3 else 1, p.name, p.message) for p in plugins)
        return view, diags
