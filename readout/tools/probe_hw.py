#!/usr/bin/env python3
"""jazzwatch hardware probe — read-only discovery for the Raspberry Pi bot-host.

Run this ON THE PI (Raspberry Pi OS / Debian, arm32). It only *reads* things
(lists devices, scans I2C, queries the PiSugar server). It changes nothing.

Goal: pin down how the Yahboom V3.0 board, the PiSugar 3, and the RPLidar are
attached so jazzwatch can read battery + accessory state reliably.

Usage:
    python3 probe_hw.py            # human-readable report
    python3 probe_hw.py --json     # machine-readable (for pasting back)

Stdlib only — no pip installs required. Optional extras (Rosmaster_Lib, smbus2,
i2c-tools) are used if present and skipped gracefully if not.
"""

from __future__ import annotations

import glob
import json
import os
import re
import shutil
import socket
import subprocess
import sys

# Known USB VID:PID hints (lowercase, no colon) → friendly name.
USB_HINTS = {
    "10c4:ea60": "CP210x UART (common on Slamtec RPLidar A1/A2)",
    "1a86:7523": "CH340 UART (common on Yahboom/STM32 boards)",
    "1a86:55d4": "CH9102 UART (Yahboom/STM32 boards)",
    "0403:6001": "FTDI FT232 UART",
    "0483:5740": "STMicroelectronics Virtual COM (STM32 CDC)",
}

# I2C addresses we can attribute up front.
I2C_HINTS = {
    "57": "PiSugar 3 battery chip",
    "68": "RTC (PiSugar RTC / DS3231-class)",
}


def run(cmd: list[str], timeout: int = 5) -> tuple[int, str, str]:
    """Run a command, never raise. Returns (rc, stdout, stderr)."""
    if shutil.which(cmd[0]) is None:
        return (127, "", f"{cmd[0]}: not installed")
    try:
        p = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False
        )
        return (p.returncode, p.stdout.strip(), p.stderr.strip())
    except Exception as exc:  # timeout / permission / anything
        return (1, "", f"{cmd[0]}: {exc}")


def probe_serial() -> list[dict]:
    """List /dev/ttyUSB* and /dev/ttyACM* candidates."""
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    results = []
    for port in ports:
        entry = {"device": port, "readable": os.access(port, os.R_OK)}
        # Try to attribute it to a USB device via the sysfs symlink.
        rc, out, _ = run(["udevadm", "info", "--query=property", f"--name={port}"])
        if rc == 0:
            vid = re.search(r"ID_VENDOR_ID=(\w+)", out)
            pid = re.search(r"ID_MODEL_ID=(\w+)", out)
            model = re.search(r"ID_MODEL=(.+)", out)
            if vid and pid:
                key = f"{vid.group(1).lower()}:{pid.group(1).lower()}"
                entry["usb_id"] = key
                entry["hint"] = USB_HINTS.get(key, "unknown device")
            if model:
                entry["model"] = model.group(1).strip()
        results.append(entry)
    return results


def probe_lsusb() -> list[str]:
    rc, out, err = run(["lsusb"])
    if rc != 0:
        return [f"(lsusb unavailable: {err})"]
    annotated = []
    for line in out.splitlines():
        m = re.search(r"ID (\w{4}:\w{4})", line)
        hint = USB_HINTS.get(m.group(1).lower()) if m else None
        annotated.append(line + (f"   <- {hint}" if hint else ""))
    return annotated


def probe_i2c() -> dict:
    """Scan I2C buses 1 and 0 with i2cdetect if available."""
    out: dict[str, object] = {}
    if shutil.which("i2cdetect") is None:
        out["error"] = "i2c-tools not installed (sudo apt install i2c-tools)"
        return out
    for bus in (1, 0):
        rc, text, err = run(["i2cdetect", "-y", str(bus)])
        if rc != 0:
            out[f"bus{bus}"] = {"error": err or "scan failed (I2C enabled?)"}
            continue
        # i2cdetect prints a grid; a real device is any two-hex token that
        # isn't the row label (which ends in ':'). '--' and 'UU' are skipped.
        detected = []
        for row in text.splitlines()[1:]:
            for tok in row.split()[1:]:
                if re.fullmatch(r"[0-7][0-9a-f]", tok):
                    detected.append(tok)
        detected = sorted(set(detected))
        out[f"bus{bus}"] = {
            "addresses": [
                {"addr": f"0x{a}", "hint": I2C_HINTS.get(a, "unattributed")}
                for a in detected
            ]
        }
    return out


def probe_rosmaster() -> dict:
    """Check for Yahboom's Rosmaster_Lib and try a battery read if a port exists."""
    result: dict[str, object] = {}
    try:
        import Rosmaster_Lib  # type: ignore  # noqa: F401

        result["installed"] = True
    except Exception:
        result["installed"] = False
        result["note"] = "Rosmaster_Lib not importable (Yahboom battery lib)."
        return result

    # Library present — attempt a non-destructive battery read on likely ports.
    from Rosmaster_Lib import Rosmaster  # type: ignore

    for port in sorted(glob.glob("/dev/ttyUSB*") + ["/dev/myserial"]):
        if not os.path.exists(port):
            continue
        try:
            bot = Rosmaster(com=port)
            bot.create_receive_threading()
            v = bot.get_battery_voltage()
            result.setdefault("readings", []).append({"port": port, "voltage_v": v})
            try:
                del bot
            except Exception:
                pass
        except Exception as exc:
            result.setdefault("errors", []).append({"port": port, "error": str(exc)})
    return result


def probe_pisugar() -> dict:
    """Query pisugar-server over UDS then TCP; report battery + model."""
    cmds = ["get model", "get battery", "get battery_v", "get battery_charging",
            "get battery_power_plugged", "get temperature"]

    def ask(sendfn) -> dict:
        data = {}
        for c in cmds:
            try:
                data[c] = sendfn(c)
            except Exception as exc:
                data[c] = f"error: {exc}"
        return data

    # 1) Unix domain socket.
    uds = "/tmp/pisugar-server.sock"
    if os.path.exists(uds):
        def send_uds(cmd: str) -> str:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(2)
            s.connect(uds)
            s.sendall((cmd + "\n").encode())
            resp = s.recv(256).decode().strip()
            s.close()
            return resp
        return {"transport": "uds", **ask(send_uds)}

    # 2) TCP 8423.
    try:
        def send_tcp(cmd: str) -> str:
            s = socket.create_connection(("127.0.0.1", 8423), timeout=2)
            s.sendall((cmd + "\n").encode())
            resp = s.recv(256).decode().strip()
            s.close()
            return resp
        test = send_tcp("get model")
        return {"transport": "tcp:8423", "get model": test,
                **{k: v for k, v in ask(send_tcp).items() if k != "get model"}}
    except Exception:
        pass

    return {"transport": None,
            "note": "pisugar-server not reachable on UDS or TCP 8423. "
                    "Install it or fall back to I2C 0x57 (see i2c section)."}


def collect() -> dict:
    return {
        "host": {
            "uname": run(["uname", "-a"])[1],
            "model": _read("/proc/device-tree/model"),
        },
        "serial_ports": probe_serial(),
        "lsusb": probe_lsusb(),
        "i2c": probe_i2c(),
        "yahboom_rosmaster": probe_rosmaster(),
        "pisugar": probe_pisugar(),
    }


def _read(path: str) -> str:
    try:
        with open(path, "rb") as fh:
            return fh.read().decode(errors="ignore").strip("\x00").strip()
    except Exception:
        return ""


def render(report: dict) -> str:
    lines = []
    add = lines.append
    add("=" * 60)
    add(" jazzwatch hardware probe")
    add("=" * 60)
    host = report["host"]
    add(f"Host   : {host.get('model') or 'unknown'}")
    add(f"Kernel : {host.get('uname')}")

    add("\n-- Serial ports (Yahboom / RPLidar candidates) --")
    if not report["serial_ports"]:
        add("  none found (/dev/ttyUSB*, /dev/ttyACM*)")
    for p in report["serial_ports"]:
        bits = [p["device"]]
        if p.get("usb_id"):
            bits.append(f"[{p['usb_id']}]")
        if p.get("hint"):
            bits.append(f"-> {p['hint']}")
        if not p["readable"]:
            bits.append("(NOT readable: add user to 'dialout' group)")
        add("  " + " ".join(bits))

    add("\n-- lsusb --")
    for line in report["lsusb"]:
        add("  " + line)

    add("\n-- I2C scan --")
    i2c = report["i2c"]
    if i2c.get("error"):
        add("  " + i2c["error"])
    for bus, info in i2c.items():
        if bus == "error":
            continue
        if isinstance(info, dict) and info.get("error"):
            add(f"  {bus}: {info['error']}")
        elif isinstance(info, dict):
            addrs = info.get("addresses", [])
            if not addrs:
                add(f"  {bus}: (no devices)")
            for a in addrs:
                add(f"  {bus}: {a['addr']}  {a['hint']}")

    add("\n-- Yahboom Rosmaster_Lib --")
    ym = report["yahboom_rosmaster"]
    add(f"  installed: {ym.get('installed')}")
    for r in ym.get("readings", []):
        add(f"  battery on {r['port']}: {r['voltage_v']} V")
    for e in ym.get("errors", []):
        add(f"  {e['port']}: {e['error']}")
    if ym.get("note"):
        add("  " + ym["note"])

    add("\n-- PiSugar 3 --")
    ps = report["pisugar"]
    add(f"  transport: {ps.get('transport')}")
    for k, v in ps.items():
        if k in ("transport", "note"):
            continue
        add(f"  {k}: {v}")
    if ps.get("note"):
        add("  " + ps["note"])

    add("\n" + "=" * 60)
    add(" Paste this back (or run with --json) so jazzwatch readers can be")
    add(" finalized for the Yahboom battery + accessory detection.")
    add("=" * 60)
    return "\n".join(lines)


def main() -> int:
    report = collect()
    if "--json" in sys.argv:
        print(json.dumps(report, indent=2, default=str))
    else:
        print(render(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
