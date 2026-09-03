"""Colour palette and state->style helpers for the jazzwatch TUI.

Colours are plain hex strings so they work both in Textual CSS and inline Rich
markup. The palette leans on a dark base with teal/purple/pink accents.
"""

from __future__ import annotations

# Base palette.
BG = "#11121b"
PANEL = "#1a1b26"
BORDER = "#2d2f45"
TEXT = "#c0caf5"
MUTED = "#565f89"
ACCENT = "#bb9af7"   # purple
TEAL = "#7dcfff"
PINK = "#f7768e"
GREEN = "#9ece6a"
AMBER = "#e0af68"
RED = "#f7768e"
BLUE = "#7aa2f7"

# System state (rov2_interfaces/SystemStatus) -> colour.
SYSTEM_COLORS = {
    0: MUTED,    # INIT
    1: TEAL,     # READY
    2: GREEN,    # ACTIVE
    3: AMBER,    # DEGRADED
    4: RED,      # FAULT
    5: MUTED,    # SHUTDOWN
}

# Plugin state (rov2_interfaces/PluginStatus) -> colour.
PLUGIN_COLORS = {
    0: MUTED,    # UNKNOWN
    1: MUTED,    # UNCONFIGURED
    2: BLUE,     # INACTIVE / idle
    3: GREEN,    # ACTIVE
    4: AMBER,    # DEGRADED
    5: RED,      # FAULT
}

# Runlog event level -> colour.
LOG_COLORS = {
    "info": TEXT,
    "good": GREEN,
    "warn": AMBER,
    "bad": RED,
}

OFFLINE_COLOR = MUTED


def system_color(state: int) -> str:
    return SYSTEM_COLORS.get(state, MUTED)


def plugin_color(state: int) -> str:
    return PLUGIN_COLORS.get(state, MUTED)


def log_color(level: str) -> str:
    return LOG_COLORS.get(level, TEXT)


def battery_color(volts: float) -> str:
    """Green/amber/red by pack voltage thresholds from config."""
    from . import config

    if volts < config.BATTERY_CRIT_V:
        return RED
    if volts < config.BATTERY_WARN_V:
        return AMBER
    return GREEN
