#!/usr/bin/env python3
"""PX4 offboard controller: flight control + Gazebo camera.

Uses TRANSIENT_LOCAL QoS (required for PX4 uXRCE-DDS status/position).
Camera uses Gazebo Transport (gz.transport), independent of ROS topics.
"""

from __future__ import annotations

import math
import os
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleRatesSetpoint,
    VehicleLocalPosition,
    VehicleStatus,
    Wind,
)

# Match working dual_drone_control QoS — VOLATILE breaks status/arming.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage, PixelFormatType
    _GZ_OK = True
except ImportError:
    GzNode = None
    GzImage = None
    PixelFormatType = None
    _GZ_OK = False


class DroneController(Node):
    MODE_OFFBOARD = 6
    ARM_CMD = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    MODE_CMD = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
    ARMING_STATE_ARMED = VehicleStatus.ARMING_STATE_ARMED
    NAV_STATE_OFFBOARD = 14
    FORCE_ARM_MAGIC = 21196.0

    def __init__(
        self,
        namespace: str,
        target_sys: int,
        drone_label: str,
        instance_id: int = 0,
        camera_topic: str = '',
        capture_dir: str = 'captures',
        sitl_force_arm: bool = True,
    ):
        node_name = f'drone_ctrl_{namespace.strip("/").replace("/", "_")}'
        super().__init__(node_name)

        self.ns = namespace
        self.target_sys = target_sys
        self.drone_label = drone_label
        self.instance_id = instance_id
        self.capture_dir = capture_dir
        self.sitl_force_arm = sitl_force_arm

        self.setpoint_count = 0
        self.nav_state = 0
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.armed = False
        self.in_offboard = False
        self.takeoff_alt = -3.0
        self.active = False
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._hold_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self._current_roll = 0.0
        self._current_pitch = 0.0
        self._current_q = [1.0, 0.0, 0.0, 0.0]
        self._manual_stream_override = False
        self.wind_north = 0.0
        self.wind_east = 0.0

        self._latest_image = None
        self._image_count = 0
        self._gz_node = None

        self.create_subscription(
            VehicleLocalPosition,
            f'{namespace}/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb,
            PX4_QOS,
        )
        self.create_subscription(
            Wind,
            f'{namespace}/fmu/out/wind',
            self._wind_cb,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleStatus,
            f'{namespace}/fmu/out/vehicle_status_v4',
            self._status_cb,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleAttitude,
            f'{namespace}/fmu/out/vehicle_attitude',
            self._attitude_cb,
            PX4_QOS,
        )

        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            f'{namespace}/fmu/in/vehicle_command',
            PX4_QOS,
        )
        self._offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'{namespace}/fmu/in/offboard_control_mode',
            PX4_QOS,
        )
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{namespace}/fmu/in/trajectory_setpoint',
            PX4_QOS,
        )
        self._rates_pub = self.create_publisher(
            VehicleRatesSetpoint,
            f'{namespace}/fmu/in/vehicle_rates_setpoint',
            PX4_QOS,
        )
        self._attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            f'{namespace}/fmu/in/vehicle_attitude_setpoint',
            PX4_QOS,
        )

        self._timer = self.create_timer(0.1, self._timer_cb)

        self.camera_topic = camera_topic or self._discover_camera_topic()
        if self.camera_topic and _GZ_OK:
            self._init_gz_camera()
            self.get_logger().info(
                f'[{self.drone_label}] Camera subscribed: {self.camera_topic}'
            )
        elif not _GZ_OK:
            self.get_logger().warn(
                f'[{self.drone_label}] gz python bindings missing — camera disabled'
            )
        else:
            self.get_logger().warn(
                f'[{self.drone_label}] No camera topic. Use gz_x500_mono_cam.'
            )

        os.makedirs(self.capture_dir, exist_ok=True)
        self.get_logger().info(
            f'[{self.drone_label}] Ready ns={namespace} sys={target_sys} id={instance_id}'
        )

    def _local_pos_cb(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_yaw = msg.heading

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _wind_cb(self, msg):
        self.wind_north = msg.windspeed_north
        self.wind_east = msg.windspeed_east

    def _status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.armed = msg.arming_state == self.ARMING_STATE_ARMED
        self.in_offboard = msg.nav_state == self.NAV_STATE_OFFBOARD

    def _attitude_cb(self, msg: VehicleAttitude):
        q = self._quat_normalize([float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])])
        self._current_q = q
        roll, pitch, yaw = self._quat_to_euler(q)
        if all(math.isfinite(v) for v in (roll, pitch, yaw)):
            self._current_roll = roll
            self._current_pitch = pitch

    @staticmethod
    def _quat_normalize(q):
        n = math.sqrt(sum(v * v for v in q))
        if n < 1e-8 or not math.isfinite(n):
            return [1.0, 0.0, 0.0, 0.0]
        return [v / n for v in q]

    @staticmethod
    def _quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]

    @staticmethod
    def _quat_conjugate(q):
        return [q[0], -q[1], -q[2], -q[3]]

    @staticmethod
    def _quat_rot_body_x(angle: float):
        half = angle * 0.5
        return [math.cos(half), math.sin(half), 0.0, 0.0]

    @staticmethod
    def _quat_rot_body_y(angle: float):
        half = angle * 0.5
        return [math.cos(half), 0.0, math.sin(half), 0.0]

    @staticmethod
    def _quat_rot_ned_x(angle: float):
        """Rotation about North axis (NED X)."""
        half = angle * 0.5
        return [math.cos(half), math.sin(half), 0.0, 0.0]

    @staticmethod
    def _quat_rot_ned_y(angle: float):
        """Rotation about East axis (NED Y)."""
        half = angle * 0.5
        return [math.cos(half), 0.0, math.sin(half), 0.0]

    @staticmethod
    def _quat_to_euler(q):
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return roll, pitch, 0.0

    def _quat_with_body_pitch(self, q_start, pitch_rad: float):
        q_start = self._quat_normalize(q_start)
        if abs(pitch_rad) < 1e-6:
            return q_start
        return self._quat_normalize(
            self._quat_multiply(q_start, self._quat_rot_body_y(pitch_rad))
        )

    def _quat_with_body_roll(self, q_start, roll_rad: float):
        q_start = self._quat_normalize(q_start)
        if abs(roll_rad) < 1e-6:
            return q_start
        return self._quat_normalize(
            self._quat_multiply(q_start, self._quat_rot_body_x(roll_rad))
        )

    def _quat_error_rates(self, q_des, q_cur, kp: float, max_rate: float):
        q_des = self._quat_normalize(q_des)
        q_cur = self._quat_normalize(q_cur)
        q_err = self._quat_multiply(q_des, self._quat_conjugate(q_cur))
        if q_err[0] < 0.0:
            q_err = [-v for v in q_err]
        w_clamped = max(-1.0, min(1.0, q_err[0]))
        angle = 2.0 * math.acos(w_clamped)
        rx, ry, rz = 2.0 * q_err[1], 2.0 * q_err[2], 2.0 * q_err[3]
        if not all(math.isfinite(v) for v in (rx, ry, rz, angle)):
            return 0.0, 0.0, 0.0, math.pi
        roll_rate = max(-max_rate, min(max_rate, kp * rx))
        pitch_rate = max(-max_rate, min(max_rate, kp * ry))
        yaw_rate = max(-max_rate, min(max_rate, kp * 0.2 * rz))
        return roll_rate, pitch_rate, yaw_rate, angle

    @staticmethod
    def _quat_rotate_vector(q, v):
        """Rotate vector v by unit quaternion q."""
        qw, qx, qy, qz = q
        vx, vy, vz = v
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return [
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        ]

    def _hover_thrust_magnitude(self, q=None) -> float:
        if q is None:
            roll, pitch = self._current_roll, self._current_pitch
        else:
            roll, pitch, _ = self._quat_to_euler(q)
        cos_tilt = max(math.cos(roll) * math.cos(pitch), 0.15)
        return min(0.95, max(0.55, 0.55 / cos_tilt))

    def _thrust_body_world_up(self, q_attitude, thrust_mag: float | None = None):
        """Thrust in body frame so net force is straight up in NED (no XY drift)."""
        if thrust_mag is None:
            thrust_mag = self._hover_thrust_magnitude(q_attitude)
        up_ned = [0.0, 0.0, -1.0]
        q_conj = self._quat_conjugate(self._quat_normalize(q_attitude))
        tb = self._quat_rotate_vector(q_conj, up_ned)
        n = math.sqrt(sum(c * c for c in tb))
        if n < 1e-6:
            return [0.0, 0.0, -thrust_mag]
        scale = thrust_mag / n
        return [tb[0] * scale, tb[1] * scale, tb[2] * scale]

    def _snapshot_attitude(self):
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.02)
        return list(self._current_q)

    def is_armed(self) -> bool:
        return self.arm_state == self.ARMING_STATE_ARMED

    def is_offboard(self) -> bool:
        return self.nav_state == self.NAV_STATE_OFFBOARD

    def _spin_for(self, duration_sec: float):
        end = time.time() + duration_sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _gz_image_cb(self, msg):
        self._latest_image = msg
        self._image_count += 1

    def _discover_camera_topic(self) -> str:
        try:
            result = subprocess.run(
                ['gz', 'topic', '-l'],
                capture_output=True,
                text=True,
                timeout=8,
                check=False,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(f'Could not list Gazebo topics: {exc}')
            return ''

        topics = [ln.strip() for ln in result.stdout.splitlines() if ln.strip()]
        needle = f'x500_mono_cam_{self.instance_id}'
        for topic in topics:
            if needle in topic and topic.endswith('/sensor/camera/image'):
                return topic
        for topic in topics:
            if '/sensor/camera/image' in topic:
                return topic
        return ''

    def _init_gz_camera(self):
        self._gz_node = GzNode()
        if not self._gz_node.subscribe(GzImage, self.camera_topic, self._gz_image_cb):
            self.get_logger().warn(
                f'[{self.drone_label}] Failed to subscribe to {self.camera_topic}'
            )
            self.camera_topic = ''

    def _timer_cb(self):
        if not self.active or self._manual_stream_override:
            return
        self._pub_offboard_mode()
        self._pub_setpoint(self._hold_x, self._hold_y, self.takeoff_alt, self._hold_yaw)
        self.setpoint_count += 1

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _pub_offboard_mode(self, body_rate: bool = False, attitude: bool = False):
        msg = OffboardControlMode()
        if attitude:
            msg.position = False
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = True
            msg.body_rate = False
        elif body_rate:
            msg.position = False
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = True
        else:
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
        msg.acceleration = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _pub_attitude_setpoint(
        self,
        q,
        thrust_z: float = -0.65,
        yaw_sp_move_rate: float = 0.0,
    ):
        q = self._quat_normalize(q)
        msg = VehicleAttitudeSetpoint()
        msg.q_d = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.thrust_body = [0.0, 0.0, float(thrust_z)]
        msg.yaw_sp_move_rate = float(yaw_sp_move_rate)
        msg.timestamp = self._now_us()
        self._attitude_pub.publish(msg)

    def _rates_for_tilt_hold(
        self,
        q_des,
        hold_x: float,
        hold_y: float,
        hold_z: float,
        kp: float = 2.5,
        max_rate: float = 1.6,
        kp_pos: float = 0.45,
    ):
        """Body rates for target tilt + P correction to kill XY drift at hold point."""
        roll_rate, pitch_rate, yaw_rate, angle = self._quat_error_rates(
            q_des, self._current_q, kp, max_rate
        )
        ex = hold_x - self.current_x
        ey = hold_y - self.current_y
        pitch_rate = max(-max_rate, min(max_rate, pitch_rate + kp_pos * ex))
        roll_rate = max(-max_rate, min(max_rate, roll_rate + kp_pos * ey))
        thrust = self._thrust_body_world_up(self._current_q)
        ez = hold_z - self.current_z
        thrust[2] = max(-0.95, min(-0.40, thrust[2] + 0.03 * ez))
        return roll_rate, pitch_rate, yaw_rate, angle, thrust

    def _maintain_tilt_at_position(
        self,
        q_des,
        hold_x: float,
        hold_y: float,
        hold_z: float,
        hold_yaw: float,
    ):
        """Hold body-Y tilt at fixed NED point: world-up thrust + rate feedback."""
        del hold_yaw
        roll_rate, pitch_rate, yaw_rate, _, thrust = self._rates_for_tilt_hold(
            q_des, hold_x, hold_y, hold_z
        )
        self._pub_offboard_mode(body_rate=True)
        self._pub_rates_setpoint(
            roll_rate, pitch_rate, yaw_rate, thrust_body=thrust
        )

    def _pub_rates_setpoint(
        self,
        roll_rate: float = 0.0,
        pitch_rate: float = 0.0,
        yaw_rate: float = 0.0,
        thrust_z: float = -0.65,
        thrust_body: list[float] | None = None,
    ):
        msg = VehicleRatesSetpoint()
        msg.roll = float(roll_rate)
        msg.pitch = float(pitch_rate)
        msg.yaw = float(yaw_rate)
        if thrust_body is not None:
            msg.thrust_body = [float(thrust_body[0]), float(thrust_body[1]), float(thrust_body[2])]
        else:
            msg.thrust_body = [0.0, 0.0, float(thrust_z)]
        msg.timestamp = self._now_us()
        self._rates_pub.publish(msg)

    def _stream_to_quaternion(
        self,
        q_des,
        hold_sec: float = 1.5,
        timeout_sec: float = 18.0,
        settle_deg: float = 8.0,
        label: str = 'attitude',
        hold_pos: tuple[float, float, float] | None = None,
        hold_yaw: float = 0.0,
    ):
        kp, max_rate = 2.0, 1.4
        q_des = self._quat_normalize(q_des)
        settle_rad = math.radians(settle_deg)
        end = time.time() + timeout_sec
        hold_start = None
        last_log = 0.0

        while time.time() < end and rclpy.ok():
            if hold_pos is not None:
                hx, hy, hz = hold_pos
                roll_rate, pitch_rate, yaw_rate, angle, thrust = self._rates_for_tilt_hold(
                    q_des, hx, hy, hz, kp, max_rate
                )
            else:
                roll_rate, pitch_rate, yaw_rate, angle = self._quat_error_rates(
                    q_des, self._current_q, kp, max_rate
                )
                thrust = self._thrust_body_world_up(self._current_q)
            self._pub_offboard_mode(body_rate=True)
            self._pub_rates_setpoint(
                roll_rate, pitch_rate, yaw_rate, thrust_body=thrust
            )
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

            now = time.time()
            if now - last_log >= 1.0:
                pos_msg = ''
                if hold_pos is not None:
                    dx = self.current_x - hold_pos[0]
                    dy = self.current_y - hold_pos[1]
                    pos_msg = f' drift=({dx:.2f},{dy:.2f})'
                self.get_logger().info(
                    f'[{self.drone_label}] {label}: pitch={math.degrees(self._current_pitch):.1f} '
                    f'err={math.degrees(angle):.1f} deg{pos_msg}'
                )
                last_log = now

            if angle < settle_rad:
                if hold_start is None:
                    hold_start = now
                if now - hold_start >= hold_sec:
                    break
            else:
                hold_start = None

    def _capture_image_while_tilted(
        self,
        q_des,
        hold_x: float,
        hold_y: float,
        hold_z: float,
        hold_yaw: float,
        filename: str | None = None,
        prefix: str = 'capture',
        timeout_sec: float = 5.0,
    ):
        """Keep tilt + position hold active while waiting for camera frame."""
        if not self.camera_topic:
            self.get_logger().warn('Camera topic not configured')
            return None

        start_count = self._image_count
        end = time.time() + timeout_sec
        while time.time() < end and rclpy.ok():
            self._maintain_tilt_at_position(q_des, hold_x, hold_y, hold_z, hold_yaw)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)
            if self._image_count > start_count and self._latest_image is not None:
                break

        if self._latest_image is None or self._image_count <= start_count:
            self.get_logger().warn('No camera frame within timeout')
            return None

        if filename:
            filepath = os.path.join(self.capture_dir, filename)
        else:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filepath = os.path.join(self.capture_dir, f'{prefix}_{timestamp}.png')
        if self._save_gz_image(self._latest_image, filepath):
            self.get_logger().info(f'[{self.drone_label}] Image saved: {filepath}')
            return filepath
        return None

    def _stream_position_for(self, x, y, z, yaw, duration_sec: float):
        end = time.time() + duration_sec
        while time.time() < end and rclpy.ok():
            self._pub_offboard_mode(body_rate=False)
            self._pub_setpoint(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def _pub_setpoint(self, x=0.0, y=0.0, z=None, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z if z is not None else self.takeoff_alt]
        msg.yaw = yaw
        msg.timestamp = self._now_us()
        self._setpoint_pub.publish(msg)

    def _send_cmd(
        self,
        command: int,
        p1=0.0,
        p2=0.0,
        p3=0.0,
        p4=0.0,
        p5=0.0,
        p6=0.0,
        p7=0.0,
    ):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.param3 = float(p3)
        msg.param4 = float(p4)
        msg.param5 = float(p5)
        msg.param6 = float(p6)
        msg.param7 = float(p7)
        msg.target_system = self.target_sys
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._cmd_pub.publish(msg)

    def _save_gz_image(self, msg, filepath: str) -> bool:
        try:
            from PIL import Image as PILImage
        except ImportError:
            self.get_logger().error('Pillow required: pip3 install Pillow')
            return False

        try:
            if msg.pixel_format_type == PixelFormatType.RGB_INT8:
                img = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
            elif msg.pixel_format_type == PixelFormatType.BGR_INT8:
                rgb = bytearray()
                for i in range(0, len(msg.data), 3):
                    rgb.extend([msg.data[i + 2], msg.data[i + 1], msg.data[i]])
                img = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(rgb))
            elif msg.pixel_format_type == PixelFormatType.RGBA_INT8:
                img = PILImage.frombytes(
                    'RGBA', (msg.width, msg.height), bytes(msg.data)
                ).convert('RGB')
            elif msg.pixel_format_type == PixelFormatType.BGRA_INT8:
                rgba = PILImage.frombytes(
                    'RGBA', (msg.width, msg.height), bytes(msg.data)
                )
                r, g, b, a = rgba.split()
                img = PILImage.merge('RGB', (r, g, b))
            else:
                self.get_logger().warn(
                    f'Unsupported pixel format: {msg.pixel_format_type}'
                )
                return False
            img.save(filepath)
            return True
        except Exception as exc:
            self.get_logger().error(f'Failed to save image: {exc}')
            return False

    def capture_image(
        self,
        prefix: str = 'capture',
        timeout_sec: float = 5.0,
        filename: str | None = None,
    ):
        if not self.camera_topic:
            self.get_logger().warn('Camera topic not configured')
            return None

        start_count = self._image_count
        end = time.time() + timeout_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._image_count > start_count and self._latest_image is not None:
                break

        if self._latest_image is None or self._image_count <= start_count:
            self.get_logger().warn('No camera frame within timeout')
            return None

        if filename:
            filepath = os.path.join(self.capture_dir, filename)
        else:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filepath = os.path.join(self.capture_dir, f'{prefix}_{timestamp}.png')
        if self._save_gz_image(self._latest_image, filepath):
            self.get_logger().info(f'[{self.drone_label}] Image saved: {filepath}')
            return filepath
        return None

    def wind_velocity(self):
        speed = math.hypot(self.wind_north, self.wind_east)
        direction_deg = math.degrees(math.atan2(self.wind_east, self.wind_north))
        return speed, direction_deg

    def start_streaming(self):
        self.active = True
        self.get_logger().info(
            f'[{self.drone_label}] Streaming ON — wait until setpoints >= 10'
        )

    def stop_streaming(self):
        self.active = False
        self.get_logger().info(f'[{self.drone_label}] Streaming OFF')

    def engage_offboard(self):
        if self.setpoint_count < 10:
            self.get_logger().warn(
                f'[{self.drone_label}] Only {self.setpoint_count} setpoints. '
                'Press "s", wait ~2s, then "o".'
            )
            return False
        self._send_cmd(self.MODE_CMD, p1=1.0, p2=float(self.MODE_OFFBOARD))
        self.get_logger().info(f'[{self.drone_label}] OFFBOARD command sent')
        return True

    def arm(self, force=None):
        if self.setpoint_count < 10:
            self.get_logger().warn(
                f'[{self.drone_label}] Need more setpoints ({self.setpoint_count}).'
            )
            return False
        use_force = self.sitl_force_arm if force is None else force
        if use_force:
            self._send_cmd(self.ARM_CMD, p1=1.0, p2=self.FORCE_ARM_MAGIC)
            self.get_logger().info(f'[{self.drone_label}] FORCE ARM sent (SITL)')
        else:
            self._send_cmd(self.ARM_CMD, p1=1.0)
            self.get_logger().info(f'[{self.drone_label}] ARM sent')
        return True

    def prepare_sitl_failsafes(self):
        """Apply SITL failsafe params once (airframe file does the real work)."""
        if getattr(self, '_sitl_params_done', False):
            return
        here = os.path.dirname(os.path.abspath(__file__))
        helper = os.path.join(here, 'apply_sitl_params.py')
        if os.path.isfile(helper):
            try:
                subprocess.run(['python3', helper, '--force'], timeout=25, check=False)
            except Exception as exc:
                self.get_logger().warn(f'SITL param helper failed: {exc}')
        self._sitl_params_done = True

    def takeoff(self, altitude_m: float = 3.0):
        """Stream setpoints, engage offboard, arm (working sequence + SITL force arm)."""
        # If already flying, just climb/hold new altitude — do not re-arm / re-param.
        alt_m = abs(altitude_m)
        if self.is_armed() and self.is_offboard() and self.active:
            self.takeoff_alt = -alt_m
            self.get_logger().info(
                f'[{self.drone_label}] Already flying — new hold alt={alt_m}m '
                f'(NED z={self.takeoff_alt})'
            )
            return True

        self.takeoff_alt = -alt_m
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._hold_yaw = 0.0

        self.prepare_sitl_failsafes()
        self.start_streaming()

        self.get_logger().info(
            f'[{self.drone_label}] Pre-arm streaming 3s (alt={alt_m}m)...'
        )
        self._spin_for(3.0)

        end = time.time() + 20.0
        last_ob = 0.0
        last_arm = 0.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            if self.setpoint_count >= 10 and not self.is_offboard() and (now - last_ob) >= 1.0:
                self.engage_offboard()
                last_ob = now

            if self.setpoint_count >= 10 and (now - last_arm) >= 1.0:
                if self.is_offboard() or (now - last_ob) >= 0.5:
                    self.arm(force=True)
                    last_arm = now

            if self.is_offboard() and self.is_armed():
                self.get_logger().info(
                    f'[{self.drone_label}] Takeoff active — climbing to {alt_m}m'
                )
                return True

        self.get_logger().warn(
            f'[{self.drone_label}] Takeoff cmds sent '
            f'(offboard={self.is_offboard()} armed={self.is_armed()} '
            f'nav={self.nav_state} setpoints={self.setpoint_count}). '
            'If stuck, in pxh run: '
            'param set NAV_DLL_ACT 0; param set COM_RCL_EXCEPT 4; param save; '
            'commander arm -f'
        )
        return self.is_offboard() and self.is_armed()

    def go_to(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Go to NED position. z must be negative for altitude (up)."""
        if z > 0:
            self.get_logger().warn(
                f'[{self.drone_label}] z={z} is positive (NED down). '
                f'Using z={-abs(z)} for altitude {abs(z)}m. '
                f'Next time enter negative z, e.g. 1 15 -10'
            )
            z = -abs(z)
        elif z == 0:
            self.get_logger().warn(
                f'[{self.drone_label}] z=0 is ground level — holding current altitude'
            )
            z = self.current_z if abs(self.current_z) > 0.2 else -3.0

        self._hold_x = x
        self._hold_y = y
        self.takeoff_alt = z
        self._hold_yaw = yaw

        if not self.active:
            self.start_streaming()

        self.get_logger().info(
            f'[{self.drone_label}] go_to x={x} y={y} z={z} yaw={yaw}'
        )

    def land(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info(f'[{self.drone_label}] LAND sent')
        # Keep offboard streaming until disarmed so PX4 can accept AUTO.LAND.

    def disarm(self, force: bool = True):
        if force:
            self._send_cmd(self.ARM_CMD, p1=0.0, p2=self.FORCE_ARM_MAGIC)
        else:
            self._send_cmd(self.ARM_CMD, p1=0.0)
        self.stop_streaming()
        self.get_logger().info(f'[{self.drone_label}] DISARM sent')

    def hold(self):
        self._hold_x = self.current_x
        self._hold_y = self.current_y
        self.takeoff_alt = self.current_z
        if not self.active:
            self.start_streaming()
        self.get_logger().info(
            f'[{self.drone_label}] HOLD at '
            f'x={self.current_x:.2f} y={self.current_y:.2f} z={self.current_z:.2f}'
        )

    def get_current_loc(self):
        return (self.current_x, self.current_y, self.current_z)

    def reached_position(self, target_x, target_y, tolerance=0.5):
        dx = self.current_x - target_x
        dy = self.current_y - target_y
        return (dx * dx + dy * dy) ** 0.5 < tolerance

    def reached_yaw(self, target_yaw: float, tolerance_deg: float = 8.0) -> bool:
        err = abs(self._wrap_pi(target_yaw - self.current_yaw))
        return err < math.radians(tolerance_deg)

    def wait_until_yaw(self, target_yaw: float, tolerance_deg: float = 8.0, timeout: float = 10.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.reached_yaw(target_yaw, tolerance_deg):
                return True
        return False

    def _capture_with_body_attitude(
        self,
        x: float,
        y: float,
        z: float,
        q_tilt_fn,
        axis_label: str,
        angle_deg: float,
        hold_sec: float = 2.5,
        filename: str | None = None,
        prefix: str = 'capture',
    ):
        """Tilt to target attitude at waypoint, hold position, capture, restore level."""
        if not self.is_armed() or not self.is_offboard():
            self.get_logger().warn(f'[{self.drone_label}] Need armed+offboard for capture')
            return None

        if z > 0:
            z = -abs(z)
        hold_x, hold_y, hold_z = x, y, z
        hold_yaw = self.current_yaw
        self._hold_x = hold_x
        self._hold_y = hold_y
        self.takeoff_alt = hold_z
        self._hold_yaw = hold_yaw

        end = time.time() + 60.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.reached_position(hold_x, hold_y, tolerance=0.35):
                break

        self._manual_stream_override = True
        path = None
        try:
            self.get_logger().info(
                f'[{self.drone_label}] Hold level at '
                f'({hold_x:.1f}, {hold_y:.1f}, {hold_z:.1f})'
            )
            self._stream_position_for(hold_x, hold_y, hold_z, hold_yaw, 1.5)

            q_start = self._snapshot_attitude()
            q_tilt = q_tilt_fn(q_start)
            self.get_logger().info(
                f'[{self.drone_label}] Capture {axis_label} {angle_deg:.0f} deg — '
                f'hold position, no XY drift'
            )
            self._stream_to_quaternion(
                q_tilt,
                hold_sec=hold_sec,
                label='capture tilt',
                hold_pos=(hold_x, hold_y, hold_z),
                hold_yaw=hold_yaw,
            )
            path = self._capture_image_while_tilted(
                q_tilt, hold_x, hold_y, hold_z, hold_yaw,
                filename=filename, prefix=prefix, timeout_sec=5.0,
            )
            self.get_logger().info(f'[{self.drone_label}] Restoring level after capture')
            self._stream_to_quaternion(
                q_start,
                hold_sec=1.0,
                label='level',
                hold_pos=(hold_x, hold_y, hold_z),
                hold_yaw=hold_yaw,
            )
            self._stream_position_for(hold_x, hold_y, hold_z, hold_yaw, 2.0)
        finally:
            self._manual_stream_override = False

        self.go_to(hold_x, hold_y, hold_z, hold_yaw)
        return path

    def _stream_yaw_at_position(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        timeout_sec: float = 12.0,
        tolerance_deg: float = 6.0,
    ) -> bool:
        """Hold NED position and yaw setpoint until yaw is reached."""
        end = time.time() + timeout_sec
        while time.time() < end and rclpy.ok():
            self._pub_offboard_mode(body_rate=False)
            self._pub_setpoint(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
            if self.reached_yaw(yaw, tolerance_deg):
                return True
        return self.reached_yaw(yaw, tolerance_deg)

    def _capture_image_while_position_hold(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        filename: str | None = None,
        prefix: str = 'capture',
        timeout_sec: float = 5.0,
    ):
        """Keep publishing position+yaw while waiting for a camera frame."""
        if not self.camera_topic:
            self.get_logger().warn('Camera topic not configured')
            return None

        start_count = self._image_count
        end = time.time() + timeout_sec
        while time.time() < end and rclpy.ok():
            self._pub_offboard_mode(body_rate=False)
            self._pub_setpoint(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)
            if self._image_count > start_count and self._latest_image is not None:
                break

        if self._latest_image is None or self._image_count <= start_count:
            self.get_logger().warn('No camera frame within timeout')
            return None

        if filename:
            filepath = os.path.join(self.capture_dir, filename)
        else:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filepath = os.path.join(self.capture_dir, f'{prefix}_{timestamp}.png')
        if self._save_gz_image(self._latest_image, filepath):
            self.get_logger().info(f'[{self.drone_label}] Image saved: {filepath}')
            return filepath
        return None

    def yaw_360_capture_every_90(
        self,
        x: float,
        y: float,
        z: float,
        filename_fn=None,
        prefix: str = 'mission',
        step_deg: float = 90.0,
        hold_sec: float = 0.8,
        clockwise: bool = True,
    ):
        """Hold XY position, yaw 360° in place, capture at each 90° (Cap 1–4)."""
        if not self.is_armed() or not self.is_offboard():
            self.get_logger().warn(f'[{self.drone_label}] Need armed+offboard for yaw scan')
            return []

        if z > 0:
            z = -abs(z)
        hold_x, hold_y, hold_z = x, y, z
        hold_yaw = self.current_yaw
        self._hold_x = hold_x
        self._hold_y = hold_y
        self.takeoff_alt = hold_z
        self._hold_yaw = hold_yaw

        end = time.time() + 60.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.reached_position(hold_x, hold_y, tolerance=0.35):
                break

        sign = -1.0 if clockwise else 1.0
        n_steps = int(360 / step_deg)
        offsets_deg = [sign * step_deg * i for i in range(n_steps)]
        cap_labels = [f'cap{i + 1}' for i in range(n_steps)]
        paths = []

        self._manual_stream_override = True
        try:
            self.get_logger().info(
                f'[{self.drone_label}] Yaw scan at ({hold_x:.1f}, {hold_y:.1f}) — '
                f'{n_steps} photos every {step_deg:.0f}°'
            )
            self._stream_position_for(hold_x, hold_y, hold_z, hold_yaw, 1.5)

            for idx, (offset_deg, cap_label) in enumerate(zip(offsets_deg, cap_labels)):
                target_yaw = self._wrap_pi(hold_yaw + math.radians(offset_deg))
                self.get_logger().info(
                    f'[{self.drone_label}] {cap_label}: yaw offset {offset_deg:.0f}° '
                    f'({idx + 1}/{n_steps})'
                )
                self._stream_yaw_at_position(hold_x, hold_y, hold_z, target_yaw)
                self._stream_position_for(hold_x, hold_y, hold_z, target_yaw, hold_sec)

                if filename_fn is not None:
                    filename = filename_fn(cap_label, offset_deg, idx)
                else:
                    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    filename = f'{prefix}_{cap_label}_{ts}.png'
                path = self._capture_image_while_position_hold(
                    hold_x, hold_y, hold_z, target_yaw,
                    filename=filename, prefix=prefix, timeout_sec=5.0,
                )
                if path:
                    paths.append(path)

            self.get_logger().info(f'[{self.drone_label}] Restoring start heading after yaw scan')
            self._stream_yaw_at_position(hold_x, hold_y, hold_z, hold_yaw)
            self._stream_position_for(hold_x, hold_y, hold_z, hold_yaw, 1.0)
        finally:
            self._manual_stream_override = False

        self.go_to(hold_x, hold_y, hold_z, hold_yaw)
        return paths

    def roll_x_360_capture_every_90(
        self,
        x: float,
        y: float,
        z: float,
        filename_fn=None,
        prefix: str = 'mission',
        hold_sec: float = 1.0,
        step_deg: float = 90.0,
    ):
        """Hold position, roll 360° about body X, capture at each 90° step."""
        if not self.is_armed() or not self.is_offboard():
            self.get_logger().warn(f'[{self.drone_label}] Need armed+offboard for roll scan')
            return []

        if z > 0:
            z = -abs(z)
        hold_x, hold_y, hold_z = x, y, z
        hold_yaw = self.current_yaw
        self._hold_x = hold_x
        self._hold_y = hold_y
        self.takeoff_alt = hold_z
        self._hold_yaw = hold_yaw

        end = time.time() + 60.0
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.reached_position(hold_x, hold_y, tolerance=0.35):
                break

        steps = [step_deg * i for i in range(1, int(360 / step_deg) + 1)]
        paths = []

        self._manual_stream_override = True
        try:
            self.get_logger().info(
                f'[{self.drone_label}] Roll scan at ({hold_x:.1f}, {hold_y:.1f}, {hold_z:.1f}) '
                f'— {len(steps)} captures every {step_deg:.0f}°'
            )
            self._stream_position_for(hold_x, hold_y, hold_z, hold_yaw, 1.5)
            q_start = self._snapshot_attitude()

            for idx, roll_deg in enumerate(steps):
                q_des = self._quat_with_body_roll(q_start, math.radians(roll_deg))
                self.get_logger().info(
                    f'[{self.drone_label}] Roll X {roll_deg:.0f}° ({idx + 1}/{len(steps)})'
                )
                self._stream_to_quaternion(
                    q_des,
                    hold_sec=hold_sec,
                    timeout_sec=28.0,
                    settle_deg=10.0,
                    label=f'roll X {roll_deg:.0f}',
                    hold_pos=(hold_x, hold_y, hold_z),
                    hold_yaw=hold_yaw,
                )
                if filename_fn is not None:
                    filename = filename_fn(roll_deg, idx)
                else:
                    ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    filename = f'{prefix}_roll{int(roll_deg)}_{ts}.png'
                path = self._capture_image_while_tilted(
                    q_des, hold_x, hold_y, hold_z, hold_yaw,
                    filename=filename, prefix=prefix, timeout_sec=5.0,
                )
                if path:
                    paths.append(path)

            self.get_logger().info(f'[{self.drone_label}] Restoring level after roll scan')
            self._stream_to_quaternion(
                q_start,
                hold_sec=1.0,
                timeout_sec=20.0,
                label='level',
                hold_pos=(hold_x, hold_y, hold_z),
                hold_yaw=hold_yaw,
            )
            self._stream_position_for(hold_x, hold_y, hold_z, hold_yaw, 1.5)
        finally:
            self._manual_stream_override = False

        self.go_to(hold_x, hold_y, hold_z, hold_yaw)
        return paths

    def capture_with_body_pitch_y(
        self,
        x: float,
        y: float,
        z: float,
        pitch_y_deg: float = 45.0,
        hold_sec: float = 2.5,
        filename: str | None = None,
        prefix: str = 'capture',
    ):
        """At waypoint: body-Y tilt, hold XY fixed, capture, level."""
        sign = 1.0 if pitch_y_deg >= 0 else -1.0
        pitch_y_deg = sign * max(25.0, min(45.0, abs(pitch_y_deg)))
        pitch_rad = math.radians(pitch_y_deg)

        def q_tilt_fn(q_start):
            return self._quat_with_body_pitch(q_start, pitch_rad)

        return self._capture_with_body_attitude(
            x, y, z, q_tilt_fn, 'pitch Y', pitch_y_deg,
            hold_sec, filename, prefix,
        )

    def capture_with_body_roll_x(
        self,
        x: float,
        y: float,
        z: float,
        roll_x_deg: float = 45.0,
        hold_sec: float = 2.5,
        filename: str | None = None,
        prefix: str = 'capture',
    ):
        """At waypoint: body-X roll, hold XY fixed, capture, level."""
        sign = 1.0 if roll_x_deg >= 0 else -1.0
        roll_x_deg = sign * max(25.0, min(45.0, abs(roll_x_deg)))
        roll_rad = math.radians(roll_x_deg)

        def q_tilt_fn(q_start):
            return self._quat_with_body_roll(q_start, roll_rad)

        return self._capture_with_body_attitude(
            x, y, z, q_tilt_fn, 'roll X', roll_x_deg,
            hold_sec, filename, prefix,
        )

    def status(self):
        wind_speed, wind_dir = self.wind_velocity()
        print(f'--- {self.drone_label} status ---')
        print(f'  namespace      : {self.ns}')
        print(f'  target_system  : {self.target_sys}')
        print(f'  instance_id    : {self.instance_id}')
        print(f'  streaming      : {self.active}')
        print(f'  nav_state      : {self.nav_state} (offboard={self.NAV_STATE_OFFBOARD})')
        print(f'  arm_state      : {self.arm_state} (armed={self.ARMING_STATE_ARMED})')
        print(f'  armed/offboard : {self.is_armed()} / {self.is_offboard()}')
        print(f'  setpoint_count : {self.setpoint_count}')
        print(
            f'  current pos    : '
            f'x={self.current_x:.2f} y={self.current_y:.2f} z={self.current_z:.2f} '
            f'yaw={math.degrees(self.current_yaw):.1f} pitch={math.degrees(self._current_pitch):.1f} deg'
        )
        print(
            f'  hold target    : x={self._hold_x:.2f} y={self._hold_y:.2f} '
            f'z={self.takeoff_alt:.2f} yaw={self._hold_yaw:.2f}'
        )
        print(f'  wind           : {wind_speed:.2f} m/s @ {wind_dir:.1f} deg')
        print(f'  camera topic   : {self.camera_topic or "not available"}')
        print(f'  frames recv    : {self._image_count}')
        print(f'  capture dir    : {os.path.abspath(self.capture_dir)}')
        print('----------------------------')

    def destroy_node(self):
        self._gz_node = None
        self._latest_image = None
        super().destroy_node()
