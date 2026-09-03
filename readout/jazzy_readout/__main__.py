"""``python -m jazzy_readout`` entry point."""

from __future__ import annotations

import argparse

from .app import JazzwatchApp


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="jazzwatch", description="mx_jazzhands readout TUI")
    parser.add_argument("--mock", action="store_true", help="use synthetic data (no ROS)")
    parser.add_argument("--no-battery", action="store_true", help="skip serial battery reads")
    parser.add_argument("--port", default=None, help="battery serial port (default: auto)")
    args = parser.parse_args(argv)

    app = JazzwatchApp(mock=args.mock, battery=not args.no_battery, port=args.port)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
