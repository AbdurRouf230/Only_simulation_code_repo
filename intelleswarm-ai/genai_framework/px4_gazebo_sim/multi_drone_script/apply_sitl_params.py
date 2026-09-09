#!/usr/bin/env python3
"""Apply SITL failsafe overrides so ROS2/uXRCE can arm without QGroundControl.

IMPORTANT: these params are INT32 in PX4. Sending REAL32 causes:
  ERROR [mavlink] param types mismatch param: NAV_DLL_ACT
and can spam/stall the mavlink link if called repeatedly.
"""
from __future__ import annotations

import os
import pathlib
import shutil
import sys
import time

HOME = pathlib.Path.home()
PX4 = pathlib.Path(os.environ.get("PX4_DIR") or (HOME / "PX4-Main/PX4-Autopilot"))
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
        print("pymavlink not installed — PX4 airframe params still apply at spawn")
        return False

    ports = [14550 + i for i in range(8)] + [14560 + i * 10 for i in range(8)]
    any_ok = False
    for port in ports:
        try:
            m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", timeout=2)
            m.wait_heartbeat(timeout=3)
            for name, val in PARAMS:
                param_type = (
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                    if name.startswith("MPC_")
                    else mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                m.mav.param_set_send(
                    m.target_system,
                    m.target_component,
                    name.encode("utf-8"),
                    float(val),
                    param_type,
                )
            print(f"set SITL arm params on UDP {port} sys={m.target_system}")
            try:
                m.close()
            except Exception:
                pass
            any_ok = True
        except Exception:
            continue
    if any_ok:
        FLAG.write_text("ok\n", encoding="utf-8")
    return any_ok


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
