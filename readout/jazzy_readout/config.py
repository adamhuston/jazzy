"""Central configuration for jazzwatch.

Tweak topic names, timeouts, and the battery gauge range here so the rest of
the app stays free of magic numbers.
"""

from __future__ import annotations

# --- ROS topics ------------------------------------------------------------
STATUS_TOPIC = "/rov2_core/status"       # rov2_interfaces/SystemStatus
DIAGNOSTICS_TOPIC = "/diagnostics"       # diagnostic_msgs/DiagnosticArray

# Consider the framework OFFLINE if no SystemStatus arrives within this window.
OFFLINE_TIMEOUT_S = 3.0

# --- UI refresh ------------------------------------------------------------
UI_REFRESH_HZ = 4.0                      # how often the panes repaint
RUNLOG_MAX_LINES = 500

# --- Battery (Yahboom pack) ------------------------------------------------
# Label: 2200 mAh 7.4 V 2S1P Li-ion -> full 8.4 V, empty 6.0 V.
BATTERY_FULL_V = 8.4
BATTERY_EMPTY_V = 6.0
BATTERY_WARN_V = 6.8                     # amber below this
BATTERY_CRIT_V = 6.3                     # red below this
BATTERY_POLL_S = 3.0                     # how often to sample the serial port
BATTERY_PORT: str | None = None         # None -> auto-detect (/dev/myserial, ...)

# --- PiSugar backup UPS ----------------------------------------------------
# PiSugar S Plus is hardware-only: presence, no battery telemetry.
PISUGAR_SOCK = "/tmp/pisugar-server.sock"
PISUGAR_TCP = ("127.0.0.1", 8423)


def battery_percent(volts: float) -> float:
    """Map pack voltage to a 0-100 charge estimate (linear, clamped)."""
    span = BATTERY_FULL_V - BATTERY_EMPTY_V
    if span <= 0:
        return 0.0
    pct = (volts - BATTERY_EMPTY_V) / span * 100.0
    return max(0.0, min(100.0, pct))
