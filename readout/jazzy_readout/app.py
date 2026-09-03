"""jazzwatch — a paned terminal dashboard for the mx_jazzhands ROS 2 system.

Panes:
  * top bar   — app title, live system-state badge, loop/heartbeat stats
  * plugins   — per-plugin health table (name / category / state / message)
  * hardware  — Yahboom pack battery gauge, PiSugar presence, USB accessories
  * runlog    — colour-coded stream of state/plugin/diagnostic events

Data comes from a background ROS bridge (or mock feed) and a background
hardware poller, so the UI never blocks on I/O.
"""

from __future__ import annotations

import socket
import time

from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, Footer, RichLog, Static

from . import config, theme
from .hardware.monitor import HardwareMonitor, HardwareSnapshot
from .ros_bridge import RosBridge, Snapshot, SystemView

HOSTNAME = socket.gethostname()


class TopBar(Horizontal):
    def compose(self) -> ComposeResult:
        yield Static("jazzwatch", id="title")
        yield Static("", id="badge")
        yield Static("", id="stats")

    def update_view(self, snap: Snapshot) -> None:
        badge = self.query_one("#badge", Static)
        stats = self.query_one("#stats", Static)
        view = snap.system
        if not snap.online or view is None:
            badge.update(Text(" OFFLINE ", style=f"bold white on {theme.OFFLINE_COLOR}"))
        else:
            color = theme.system_color(view.state)
            badge.update(Text(f" {view.state_label} ", style=f"bold black on {color}"))
        t = Text()
        if view is not None:
            t.append(f"loop {view.loop_count}", style=theme.MUTED)
            t.append(f"  {view.loop_period_ms:.0f}ms", style=theme.MUTED)
            t.append(f"  jitter {view.loop_jitter_ms:.1f}ms", style=theme.MUTED)
        t.append(f"   {HOSTNAME}", style=theme.TEAL)
        t.append(f"  {time.strftime('%H:%M:%S')}", style=theme.TEXT)
        t.append(f"  [{snap.source}]", style=theme.ACCENT)
        stats.update(t)


class PluginsTable(DataTable):
    def on_mount(self) -> None:
        self.cursor_type = "row"
        self.zebra_stripes = True
        self.add_columns("plugin", "category", "state", "message")

    def update_plugins(self, view: SystemView | None) -> None:
        self.clear()
        if view is None:
            return
        for p in view.plugins:
            color = theme.plugin_color(p.state)
            self.add_row(
                Text(p.name, style=theme.TEXT),
                Text(p.category, style=theme.MUTED),
                Text(p.state_label, style=f"bold {color}"),
                Text(p.message or "-", style=theme.MUTED),
            )


class BatteryGauge(Static):
    WIDTH = 22

    def update_battery(self, volts: float | None, err: str | None) -> None:
        if volts is None:
            self.update(Text(f"Yahboom  {err or 'no data'}", style=theme.MUTED))
            return
        pct = config.battery_percent(volts)
        color = theme.battery_color(volts)
        filled = int(round(pct / 100 * self.WIDTH))
        bar = "█" * filled + "░" * (self.WIDTH - filled)
        t = Text()
        t.append("Yahboom  ", style=theme.TEXT)
        t.append(bar, style=color)
        t.append(f"  {pct:3.0f}%  {volts:.1f} V", style=f"bold {color}")
        self.update(t)


class HardwarePanel(Vertical):
    def compose(self) -> ComposeResult:
        yield BatteryGauge(id="battery")
        yield Static("", id="pisugar")
        yield Static("", id="accessories")

    def update_hw(self, hw: HardwareSnapshot) -> None:
        self.query_one("#battery", BatteryGauge).update_battery(hw.battery_volts, hw.battery_error)

        ps = self.query_one("#pisugar", Static)
        if hw.pisugar and hw.pisugar.present:
            t = Text("PiSugar  ", style=theme.TEXT)
            t.append("● ", style=theme.GREEN)
            t.append(f"{hw.pisugar.model}  {hw.pisugar.note}", style=theme.MUTED)
        else:
            t = Text("PiSugar  ", style=theme.TEXT)
            t.append("○ ", style=theme.MUTED)
            t.append("not detected", style=theme.MUTED)
        ps.update(t)

        acc = self.query_one("#accessories", Static)
        block = Text("Accessories\n", style=f"bold {theme.ACCENT}")
        if not hw.accessories:
            block.append("  none\n", style=theme.MUTED)
        for a in hw.accessories:
            block.append(f"  {a.device}  ", style=theme.TEAL)
            block.append(f"{a.vid_pid}  ", style=theme.MUTED)
            block.append(f"{a.label}\n", style=theme.TEXT)
        acc.update(block)


class JazzwatchApp(App):
    CSS = """
    Screen { background: #11121b; }

    TopBar {
        height: 3;
        background: #1a1b26;
        border-bottom: solid #2d2f45;
        padding: 0 1;
    }
    #title { width: auto; color: #bb9af7; text-style: bold; content-align: left middle; }
    #badge { width: auto; content-align: center middle; padding: 0 2; }
    #stats { width: 1fr; content-align: right middle; }

    #body { height: 1fr; }

    PluginsTable {
        width: 2fr;
        background: #1a1b26;
        border: round #2d2f45;
        border-title-color: #7dcfff;
    }
    HardwarePanel {
        width: 1fr;
        background: #1a1b26;
        border: round #2d2f45;
        border-title-color: #7dcfff;
        padding: 1;
    }
    HardwarePanel > Static { margin-bottom: 1; }

    #runlog {
        height: 14;
        background: #1a1b26;
        border: round #2d2f45;
        border-title-color: #7dcfff;
    }
    """

    BINDINGS = [
        ("q", "quit", "Quit"),
        ("c", "clear_log", "Clear log"),
    ]

    def __init__(self, mock: bool = False, battery: bool = True, port: str | None = None):
        super().__init__()
        self.bridge = RosBridge(mock=mock)
        self.hw = HardwareMonitor(port=port, enable_battery=battery)
        self._log_shown = 0

    def compose(self) -> ComposeResult:
        yield TopBar()
        with Horizontal(id="body"):
            table = PluginsTable()
            table.border_title = "plugins"
            yield table
            panel = HardwarePanel()
            panel.border_title = "hardware"
            yield panel
        log = RichLog(id="runlog", highlight=False, markup=False, wrap=True)
        log.border_title = "runlog"
        yield log
        yield Footer()

    def on_mount(self) -> None:
        self.bridge.start()
        self.hw.start()
        self.set_interval(1.0 / config.UI_REFRESH_HZ, self.refresh_data)

    def refresh_data(self) -> None:
        snap = self.bridge.snapshot()
        hw = self.hw.snapshot()
        self.query_one(TopBar).update_view(snap)
        self.query_one(PluginsTable).update_plugins(snap.system)
        self.query_one(HardwarePanel).update_hw(hw)
        self._drain_log(snap)

    def _drain_log(self, snap: Snapshot) -> None:
        new = snap.log_seq - self._log_shown
        if new <= 0:
            return
        log = self.query_one("#runlog", RichLog)
        for ev in snap.log[-min(new, len(snap.log)):]:
            line = Text(time.strftime("%H:%M:%S", time.localtime(ev.ts)) + "  ", style=theme.MUTED)
            line.append(ev.text, style=theme.log_color(ev.level))
            log.write(line)
        self._log_shown = snap.log_seq

    def action_clear_log(self) -> None:
        self.query_one("#runlog", RichLog).clear()

    def on_unmount(self) -> None:
        self.bridge.stop()
        self.hw.stop()
