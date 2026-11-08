"""USB serial ports of the arms: detection by unplugging / replugging, and access permissions."""

import getpass
import grp
import os
import subprocess
import time

from lerobot.scripts.lerobot_find_port import find_available_ports

from so101.prompts import header


def serial_ports() -> set[str]:
    return set(find_available_ports())


def wait_for_port_change(baseline: set[str], expect: str, timeout: float = 10.0) -> list[str]:
    """Poll until a port disappears from / appears in `baseline` ('removed' or 'added')."""
    deadline = time.monotonic() + timeout
    while True:
        current = serial_ports()
        diff = baseline - current if expect == "removed" else current - baseline
        if diff or time.monotonic() > deadline:
            return sorted(diff)
        time.sleep(0.2)


def detect_port(role: str) -> str:
    """Detect the port of one arm by unplugging and replugging its USB cable."""
    while True:
        before = serial_ports()
        input(f"\nUnplug the USB cable of the {role.upper()} arm and press ENTER...")
        removed = wait_for_port_change(before, "removed", timeout=5.0)
        if len(removed) != 1:
            print(f"Could not detect the port (ports removed: {removed or 'none'}). Plug it back in and try again.")
            input("Press ENTER once the cable is plugged back in...")
            continue

        without = serial_ports()
        input(f"Detected {removed[0]}. Plug the {role.upper()} arm back in and press ENTER...")
        added = wait_for_port_change(without, "added", timeout=10.0)
        if len(added) != 1:
            print(f"Could not detect the port after replugging (ports added: {added or 'none'}). Try again.")
            continue

        port = added[0]
        print(f"The {role} arm is on {port}")
        return port


def dialout_hint() -> str:
    """Explain how to get permanent port access through the `dialout` group, depending on the current state."""
    try:
        group = grp.getgrnam("dialout")
    except KeyError:
        return ""
    user = getpass.getuser()
    if group.gr_gid in os.getgroups():
        return ""
    if user in group.gr_mem:
        return "You are in the 'dialout' group, but this session predates it: log out and back in (or reboot)."
    return (
        "Tip: add yourself to the 'dialout' group once to stop needing sudo here:\n"
        "  sudo usermod -aG dialout $USER   (then log out and back in)"
    )


def ensure_port_access(port: str) -> None:
    """Make sure the current user can read/write the port.

    Members of the `dialout` group already have access, so nothing happens. Otherwise this falls back to
    `sudo chmod 666 <port>`, which lasts until the arm is replugged or the PC reboots.
    """
    if os.access(port, os.R_OK | os.W_OK):
        print(f"Access to {port}: OK")
        return
    print(f"No read/write access to {port}, running: sudo chmod 666 {port}")
    if hint := dialout_hint():
        print(hint)
    subprocess.run(["sudo", "chmod", "666", port], check=True)
    if not os.access(port, os.R_OK | os.W_OK):
        raise PermissionError(f"Still no access to {port} after chmod.")


def detect_arm_ports() -> tuple[str, str]:
    header("Detecting USB ports")
    input("Make sure BOTH arms are powered and connected over USB, then press ENTER...")

    leader_port = detect_port("leader")
    follower_port = detect_port("follower")
    if leader_port == follower_port:
        raise RuntimeError(f"Leader and follower were detected on the same port ({leader_port}).")

    for port in (leader_port, follower_port):
        ensure_port_access(port)
    return leader_port, follower_port
