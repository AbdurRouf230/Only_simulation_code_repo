"""Thread-safe wrapper for rclpy.spin_once (multi-drone fleet runner).

ROS 2 Humble rclpy is not safe when several threads call spin_once() at once,
even on different nodes — you get: ValueError: generator already executing

Import and call install_thread_safe_spin() once after rclpy.init().
"""

from __future__ import annotations

import threading

import rclpy

_installed = False
_lock = threading.Lock()
_original_spin_once = rclpy.spin_once


def install_thread_safe_spin() -> None:
    global _installed
    if _installed:
        return

    def _thread_safe_spin_once(node, *args, **kwargs):
        with _lock:
            return _original_spin_once(node, *args, **kwargs)

    rclpy.spin_once = _thread_safe_spin_once  # type: ignore[method-assign]
    _installed = True
