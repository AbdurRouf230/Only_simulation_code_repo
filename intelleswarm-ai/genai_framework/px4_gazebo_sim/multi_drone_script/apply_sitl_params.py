#!/usr/bin/env python3
"""Apply SITL failsafe overrides so ROS2/uXRCE can arm without QGroundControl.

IMPORTANT: these params are INT32 in PX4. Sending REAL32 causes:
  ERROR [mavlink] param types mismatch param: NAV_DLL_ACT
and can spam/stall the mavlink link if called repeatedly.
"""
from __future__ import annotations

import pathlib
import shutil
import sys
import time

HOME = pathlib.Path.home()
PX4 = HOME / "PX4-Main/PX4-Autopilot"
SRC = PX4 / "ROMFS/px4fmu_common/init.d-posix/airframes"
DST = PX4 / "build/px4_sitl_default/etc/init.d-posix/airframes"
FLAG = pathlib.Path("/tmp/multidrone_sitl_params_ok")

# All of these are integer params in PX4.
PARAMS = [
    ("NAV_DLL_ACT", 0),
    ("NAV_RCL_ACT", 0),
    ("COM_RCL_EXCEPT", 4),
    ("COM_ARM_WO_GPS", 1),
    ("CBRK_IO_SAFETY", 22027),
    ("MPC_TILTMAX_AIR", 89),
]


def copy_airframes():
    for name in ["4010_gz_x500_mono_cam", "4014_gz_x500_mono_cam_down"]:
        s = SRC / name
        d = DST / name
        if s.exists() and DST.exists():
            shutil.copy2(s, d)


def set_live_params():
    try:
        from pymavlink import mavutil
    except ImportError:
        print("pymavlink not installed. In pxh run:")
        for n, v in PARAMS:
            print(f"  param set {n} {v}")
        print("  param save")
        return False

    # Use GCS port (14550). Do NOT bind onboard 14540 during flight control.
    print("Setting SITL params via MAVLink UDP 14550 (INT32)...")
    try:
        m = mavutil.mavlink_connection("udpin:0.0.0.0:14550", timeout=5)
        m.wait_heartbeat(timeout=8)
        for name, val in PARAMS:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32 if name.startswith("MPC_") else mavutil.mavlink.MAV_PARAM_TYPE_INT32
            m.mav.param_set_send(
                m.target_system,
                m.target_component,
                name.encode("utf-8"),
                float(val),
                param_type,
            )
            print("set", name, val)
            time.sleep(0.05)
        # Close so we don't keep holding the GCS mavlink port.
        try:
            m.close()
        except Exception:
            pass
        FLAG.write_text("ok\n", encoding="utf-8")
        return True
    except Exception as exc:
        print("live param set skipped:", exc)
        print("Fallback — in pxh run: param set NAV_DLL_ACT 0; param set COM_RCL_EXCEPT 4; param save")
        return False


def main():
    force = "--force" in sys.argv
    if FLAG.exists() and not force:
        print("SITL params already applied this boot (use --force to re-apply)")
        return 0
    copy_airframes()
    set_live_params()
    return 0


if __name__ == "__main__":
    sys.exit(main())
