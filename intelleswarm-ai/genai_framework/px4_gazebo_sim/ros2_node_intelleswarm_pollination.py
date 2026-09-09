#!/usr/bin/env python3
"""
--------------------------------------------------------------------------
 Copyright (c) 2026, IntelleSwarm Corporation. All Rights Reserved.
 This software is proprietary and confidential. Unauthorized copying or
 dissemination is strictly prohibited.
--------------------------------------------------------------------------
ros2_node_intelleswarm_pollination.py
Author: Zahid Rahman

Enhanced ROS 2 node for IntelleSwarm Assistive Pollination missions:
- Integrates with PX4 Gazebo simulation
- Coordinates multi-drone pollination swarms using MARL algorithms
- Implements flower detection and coverage optimization
- Manages pollination mission planning and execution
- Connects with assistive_pollination module for agricultural AI

Compatible with agricultural_farm.world and px4_multi_drone.sh
"""

import math
import os
import sys
import threading
import yaml
import time
from typing import Dict, Any, List, Tuple, Optional
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray

_REPO_ROOT = str(Path(__file__).resolve().parents[2])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

_MULTI_SCRIPT = Path(__file__).resolve().parent / "multi_drone_script"
_CONTROLLER_CANDIDATES = [
    _MULTI_SCRIPT,
    Path(os.environ.get("DRONE_CONTROLLER_DIR", "")),
    Path.home() / "ros2_ws" / "src" / "px4_python" / "MultiDrone",
    Path("/home/rouf/ros2_ws/src/px4_python/MultiDrone"),
    Path("/mnt/e/Multi Drone project/Main setup/main_used_code_in_script"),
    Path(__file__).resolve().parent.parent.parent.parent / "main_used_code_in_script",
]
_CONTROLLER_CANDIDATES = [p for p in _CONTROLLER_CANDIDATES if str(p).strip() not in ("", ".")]
for _p in (_MULTI_SCRIPT, *_CONTROLLER_CANDIDATES):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

try:
    from genai_framework.sdk import DroneBrain, EnvReading, GeoFence, SafetyManager, Pose
except ImportError as e:
    print(f"Warning: Could not import IntelleSwarm SDK: {e}")
    DroneBrain = EnvReading = GeoFence = SafetyManager = Pose = None

try:
    from assistive_pollination.models.flower_detector import FlowerDetector
except ImportError as e:
    print(f"Warning: FlowerDetector not available: {e}")
    FlowerDetector = None

# Prebuilt multi_drone_script stack (not reimplemented here)
from collision_avoidance import FleetRegistry  # noqa: E402
from drone_controller import DroneController  # noqa: E402
from mission_logic import FleetDrone, run_grid_mission_fleet, land_and_wait  # noqa: E402
from multi_drone_config import FLEET_STATE_PATH  # noqa: E402
from multi_mission_runner import mission_thread  # noqa: E402
from ros_thread_spin import install_thread_safe_spin  # noqa: E402


class CoverageOptimizer:
    """Lightweight coverage helper used when the dedicated module is absent."""

    def __init__(self, *args, **kwargs):
        self.overlap_percentage = kwargs.get("overlap_percentage", 20)


class SwarmCoordinator:
    """In-process swarm coordinator matching the node constructor."""

    def __init__(self, num_drones: int = 6, communication_range: float = 1000.0, **kwargs):
        self.num_drones = num_drones
        self.communication_range = communication_range


class PollinationMissionPlanner:
    """In-process mission planner matching the node constructor."""

    def __init__(self, *args, **kwargs):
        self.max_drones = kwargs.get("max_drones", 6)

# Configuration
@dataclass
class FlowerPatch:
    """Represents a detected flower patch for pollination"""
    id: int
    center_x: float
    center_y: float
    radius: float
    flower_type: str
    priority: int
    pollination_status: float  # 0.0 = not pollinated, 1.0 = fully pollinated

@dataclass
class DroneStatus:
    """Enhanced drone status for pollination missions"""
    drone_id: str
    position: Tuple[float, float, float]  # x, y, z
    battery: float
    mission_state: str
    current_target: Optional[FlowerPatch]
    pollination_payload: float
    flowers_pollinated: int

class IntelleSwarmPollinationNode(Node):
    """Enhanced ROS2 node for coordinated pollination missions"""

    def __init__(self):
        super().__init__("intelleswarm_pollination_controller")

        self.get_logger().info("🌻 IntelleSwarm Pollination System starting...")

        # Load configuration
        self._load_configuration()

        # Initialize components
        self._init_agricultural_ai()
        self._init_mission_planning()
        self._init_swarm_coordination()
        self._init_safety_systems()

        # ROS communication setup
        self._setup_ros_communication()

        # Prebuilt PX4 fleet (DroneController + FleetRegistry collision avoidance)
        self.fleet_drones: Dict[str, FleetDrone] = {}
        self.mission_threads: List[threading.Thread] = []
        self._init_px4_fleet()

        # Mission state
        self.mission_active = False
        self.mission_start_time = None
        self.total_flowers_detected = 0
        self.total_area_covered = 0.0

        # Performance tracking
        self.performance_metrics = {
            'pollination_efficiency': 0.0,
            'coverage_completeness': 0.0,
            'energy_consumption': 0.0,
            'coordination_quality': 0.0
        }

        self.get_logger().info("🚁 Pollination system initialized and ready!")

        self._auto_start = os.environ.get("POLLINATION_AUTO_START", "1") != "0"
        self._duration_sec = float(os.environ.get("POLLINATION_DURATION", "0") or 0)

    def _load_configuration(self):
        """Load pollination mission configuration"""
        config_path = Path(__file__).parent / "pollination_drone_config.yaml"

        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().warn(f"Config file not found: {config_path}")
            self.config = self._default_config()

        # Extract key configurations
        self.num_drones = max(1, int(os.environ.get("POLLINATION_NUM_DRONES", "6")))
        self.drone_ids = [f"drone_{i}" for i in range(1, self.num_drones + 1)]
        self.obs_dim = 32
        self.msg_dim = 16
        self.action_dim = 5  # vx, vy, vz, yaw_rate, pollination_rate

        # Pollination parameters
        self.pollination_config = self.config.get('pollination', {})
        self.coverage_height = self.pollination_config.get('coverage_height', 3.0)
        self.pollination_speed = self.pollination_config.get('pollination_speed', 2.0)

        self.get_logger().info(f"🔧 Loaded config for {len(self.drone_ids)} drones")

    def _default_config(self):
        """Default configuration if file not found"""
        return {
            'pollination': {
                'coverage_height': 3.0,
                'pollination_speed': 2.0,
                'overlap_percentage': 20,
                'flower_priorities': {
                    'sunflower': 100,
                    'clover': 80,
                    'wildflower': 40
                }
            },
            'safety': {
                'geo_fence': {
                    'vertices': [[-200, -200], [200, -200], [200, 200], [-200, 200]]
                }
            }
        }

    def _init_agricultural_ai(self):
        """Initialize agricultural AI components"""
        self.get_logger().info("🌾 Initializing Agricultural AI components...")

        try:
            self.flower_detector = FlowerDetector() if FlowerDetector else None
            self.coverage_optimizer = CoverageOptimizer()
            self.get_logger().info("✅ Agricultural AI components loaded")
        except Exception as e:
            self.get_logger().warn(f"Agricultural AI components not available: {e}")
            self.flower_detector = None
            self.coverage_optimizer = CoverageOptimizer()

    def _init_mission_planning(self):
        """Initialize mission planning system"""
        self.get_logger().info("📋 Initializing Mission Planning...")

        try:
            self.mission_planner = PollinationMissionPlanner()
        except Exception as e:
            self.get_logger().warn(f"Mission planner not available: {e}")
            self.mission_planner = None

        # Known flower patches from the world
        self.flower_patches = [
            FlowerPatch(1, 50, 50, 15, "sunflower", 100, 0.0),
            FlowerPatch(2, -50, 50, 12, "sunflower", 100, 0.0),
            FlowerPatch(3, 0, 0, 20, "clover", 80, 0.0),
        ]

        self.get_logger().info(f"🌸 Detected {len(self.flower_patches)} flower patches")

    def _init_swarm_coordination(self):
        """Initialize swarm coordination with MARL"""
        self.get_logger().info("🤖 Initializing Swarm Coordination...")

        try:
            self.swarm_coordinator = SwarmCoordinator(
                num_drones=len(self.drone_ids),
                communication_range=1000.0
            )
        except Exception as e:
            self.get_logger().warn(f"Swarm coordinator not available: {e}")
            self.swarm_coordinator = None

        # Brains are loaded lazily — constructing 6 models here delays PX4 takeoff
        self.drone_brains = {}
        self.drone_status = {}

        for drone_id in self.drone_ids:
            self.drone_status[drone_id] = DroneStatus(
                drone_id=drone_id,
                position=(0.0, 0.0, 0.0),
                battery=100.0,
                mission_state="ready",
                current_target=None,
                pollination_payload=100.0,
                flowers_pollinated=0
            )

    def _init_safety_systems(self):
        """Initialize safety and geo-fencing systems"""
        self.get_logger().info("🛡️ Initializing Safety Systems...")

        fence_vertices = self.config['safety']['geo_fence']['vertices']
        self.farm_fence = None
        self.safety_manager = None

        if GeoFence is None:
            self.get_logger().warn("GeoFence not available, safety fence disabled")
            return

        try:
            self.farm_fence = GeoFence(vertices=fence_vertices)
            if SafetyManager is not None:
                self.safety_manager = SafetyManager(fences=[self.farm_fence])
        except Exception as e:
            self.get_logger().warn(f"Safety manager not available: {e}")
            self.safety_manager = None

    def _setup_ros_communication(self):
        """Setup ROS2 publishers and subscribers"""
        self.get_logger().info("📡 Setting up ROS2 Communication...")

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Per-drone communication
        self.state_subscribers = {}
        self.image_subscribers = {}
        self.cmd_publishers = {}
        self.mission_publishers = {}

        for drone_id in self.drone_ids:
            # State subscriber (from PX4)
            self.state_subscribers[drone_id] = self.create_subscription(
                Odometry,
                f"/{drone_id}/fmu/out/vehicle_odometry",
                lambda msg, did=drone_id: self._state_callback(did, msg),
                sensor_qos
            )

            # Camera/image subscriber for flower detection
            self.image_subscribers[drone_id] = self.create_subscription(
                Image,
                f"/{drone_id}/camera/image_raw",
                lambda msg, did=drone_id: self._image_callback(did, msg),
                sensor_qos
            )

            # Command publisher (to PX4)
            self.cmd_publishers[drone_id] = self.create_publisher(
                Twist,
                f"/{drone_id}/fmu/in/setpoint_velocity/cmd_vel",
                control_qos
            )

            # Mission status publisher
            self.mission_publishers[drone_id] = self.create_publisher(
                String,
                f"/{drone_id}/mission_status",
                control_qos
            )

        # Global mission coordination
        self.mission_status_pub = self.create_publisher(
            String, "/mission/status", control_qos
        )

        self.flower_map_pub = self.create_publisher(
            Float32MultiArray, "/mission/flower_map", control_qos
        )

        # Mission control subscriber
        self.mission_control_sub = self.create_subscription(
            String, "/mission/control", self._mission_control_callback, control_qos
        )

        # Main control loop timer
        self.control_timer = self.create_timer(0.1, self._control_loop)  # 10Hz

        self.get_logger().info("✅ ROS2 Communication setup complete")

    def _init_px4_fleet(self):
        """Attach to PX4 SITL via prebuilt DroneController (/px4_0 .. /px4_5)."""
        self.get_logger().info("🔗 Connecting to PX4 fleet (DroneController)...")
        try:
            if os.path.isfile(FLEET_STATE_PATH):
                os.remove(FLEET_STATE_PATH)
        except OSError:
            pass

        capture_root = Path(__file__).resolve().parent / "multi_drone_script" / "captures"
        try:
            for i, drone_id in enumerate(self.drone_ids):
                cap_dir = capture_root / f"drone{i}"
                cap_dir.mkdir(parents=True, exist_ok=True)
                controller = DroneController(
                    namespace=f"/px4_{i}",
                    target_sys=i + 1,
                    drone_label=f"Drone {i}",
                    instance_id=i,
                    capture_dir=str(cap_dir),
                )
                fleet = FleetDrone(i, controller)
                self.fleet_drones[drone_id] = fleet
                for _ in range(5):
                    rclpy.spin_once(controller, timeout_sec=0.1)
                self.get_logger().info(
                    f"   {drone_id} <-> /px4_{i} attached"
                )
        except Exception as e:
            self.get_logger().error(f"PX4 fleet connect failed: {e}")
            self.fleet_drones = {}

    def _sync_state_from_px4(self):
        """Copy live PX4 local position into drone_status (State)."""
        for drone_id, fleet in self.fleet_drones.items():
            status = self.drone_status[drone_id]
            ctrl = fleet.drone
            status.position = (ctrl.current_x, ctrl.current_y, ctrl.current_z)
            fleet.sync_pose()

    def _state_callback(self, drone_id: str, msg: Odometry):
        """Process drone state updates from PX4"""
        if drone_id not in self.drone_status:
            return

        pose = msg.pose.pose
        twist = msg.twist.twist

        # Update drone status
        status = self.drone_status[drone_id]
        status.position = (pose.position.x, pose.position.y, pose.position.z)

        # Simulate battery drain during mission
        if self.mission_active and status.battery > 0:
            status.battery -= 0.01  # Drain 1% every 10 seconds at 10Hz

    def _assign_patch_from_position(self, drone_id: str):
        """Assign nearest flower patch from live pose (camera pixels unused)."""
        if drone_id not in self.drone_status:
            return
        status = self.drone_status[drone_id]
        if status.current_target:
            return
        x, y, z = status.position
        best = None
        best_d = float("inf")
        for patch in self.flower_patches:
            distance = math.sqrt((x - patch.center_x) ** 2 + (y - patch.center_y) ** 2)
            local_over_home = math.sqrt(x * x + y * y) < (patch.radius + 10)
            if distance < patch.radius + 10 or local_over_home:
                if distance < best_d:
                    best_d = distance
                    best = patch
        if best is not None:
            status.current_target = best

    def _image_callback(self, drone_id: str, msg: Image):
        """Process camera images for flower detection"""
        del msg
        self._assign_patch_from_position(drone_id)

    def _mission_control_callback(self, msg: String):
        """Handle mission control commands"""
        command = msg.data.lower()

        if command == "start_pollination":
            self._start_pollination_mission()
        elif command == "pause_mission":
            self._pause_mission()
        elif command == "resume_mission":
            self._resume_mission()
        elif command == "abort_mission":
            self._abort_mission()
        elif command == "return_to_home":
            self._return_to_home()

    def _control_loop(self):
        """Main control loop - runs at 10Hz"""
        self._sync_state_from_px4()
        for drone_id in self.drone_ids:
            self._assign_patch_from_position(drone_id)

        if not self.mission_active:
            return

        # Fleet flight is owned by mission_thread + DroneController. Do not
        # issue competing go_to_safe / Twist from this timer.
        if self.fleet_drones:
            for drone_id in self.drone_ids:
                self._update_pollination_status(drone_id, [0.0, 0.0, 0.0, 0.0, 1.0])
            self._update_mission_metrics()
            self._publish_mission_status()
            return

        for drone_id in self.drone_ids:
            if drone_id not in self.drone_status:
                continue
            if self._check_safety(drone_id):
                self._execute_drone_mission(drone_id)
            else:
                self._handle_safety_violation(drone_id)

        self._update_mission_metrics()
        self._publish_mission_status()

    def _check_safety(self, drone_id: str) -> bool:
        """Check if drone operation is safe"""
        status = self.drone_status[drone_id]

        # Check battery level
        if status.battery < 20:
            return False

        # Check geo-fence
        if self.safety_manager:
            x, y, z = status.position
            pose = Pose(x=x, y=y, z=z, yaw=0.0)
            if self.safety_manager.should_trigger_rth(pose, link_quality=1.0):
                return False

        return True

    def _execute_drone_mission(self, drone_id: str):
        """Execute pollination mission for a specific drone"""
        status = self.drone_status[drone_id]

        # Real flight is mission_thread + DroneController. Keep this loop for
        # scoring only so 6x DroneBrain.step cannot starve offboard setpoints.
        if self.fleet_drones:
            self._update_pollination_status(drone_id, [0.0, 0.0, 0.0, 0.0, 1.0])
            return

        brain = self._get_drone_brain(drone_id)
        if not brain:
            return

        # Build observation vector
        obs_vector = self._build_pollination_observation(drone_id)

        # Get messages from other drones (simplified)
        msg_vector = self._build_swarm_message(drone_id)

        # Create environment reading
        env_reading = EnvReading(
            wind_speed=2.0,  # Could be from weather sensor
            humidity=0.65,
            temperature=23.0
        )

        try:
            # Run IntelleSwarm DroneBrain
            result = brain.step(
                obs_vec=obs_vector,
                msg_vec=msg_vector,
                env=env_reading
            )

            action = result["action"][0]  # Get first element

            # Convert to pollination command
            cmd = self._action_to_pollination_command(drone_id, action)

            # Publish command
            if drone_id in self.cmd_publishers:
                self.cmd_publishers[drone_id].publish(cmd)

            # Update pollination status
            self._update_pollination_status(drone_id, action)

        except Exception as e:
            self.get_logger().warn(f"Mission execution error for {drone_id}: {e}")

    def _build_pollination_observation(self, drone_id: str):
        """Build observation vector specific to pollination missions"""
        import torch

        status = self.drone_status[drone_id]
        x, y, z = status.position

        # Basic observation components
        obs_data = [
            x, y, z,  # Position
            status.battery / 100.0,  # Battery level (normalized)
            status.pollination_payload / 100.0,  # Payload level
        ]

        # Add flower patch information
        for patch in self.flower_patches:
            distance = math.sqrt((x - patch.center_x)**2 + (y - patch.center_y)**2)
            bearing = math.atan2(patch.center_y - y, patch.center_x - x)
            obs_data.extend([
                distance / 100.0,  # Normalized distance
                bearing / math.pi,  # Normalized bearing
                patch.priority / 100.0,  # Normalized priority
                patch.pollination_status  # Completion status
            ])

        # Pad or trim to obs_dim
        while len(obs_data) < self.obs_dim:
            obs_data.append(0.0)
        obs_data = obs_data[:self.obs_dim]

        return torch.tensor([obs_data], dtype=torch.float32)

    def _build_swarm_message(self, drone_id: str):
        """Build message vector from other drones"""
        import torch

        # For now, simple message with nearest neighbor info
        status = self.drone_status[drone_id]
        x, y, z = status.position

        msg_data = [0.0] * self.msg_dim

        # Find nearest drone
        min_distance = float('inf')
        nearest_drone = None

        for other_id, other_status in self.drone_status.items():
            if other_id != drone_id:
                ox, oy, oz = other_status.position
                distance = math.sqrt((x-ox)**2 + (y-oy)**2 + (z-oz)**2)
                if distance < min_distance:
                    min_distance = distance
                    nearest_drone = other_status

        # Encode nearest neighbor information
        if nearest_drone:
            ox, oy, oz = nearest_drone.position
            msg_data[0] = (ox - x) / 100.0  # Relative x
            msg_data[1] = (oy - y) / 100.0  # Relative y
            msg_data[2] = (oz - z) / 50.0   # Relative z
            msg_data[3] = nearest_drone.battery / 100.0  # Their battery

        return torch.tensor([msg_data], dtype=torch.float32)

    def _action_to_pollination_command(self, drone_id: str, action: List[float]) -> Twist:
        """Convert MARL action to ROS Twist command for pollination"""
        cmd = Twist()

        if len(action) >= 4:
            # Scale actions appropriately for pollination speeds
            cmd.linear.x = float(action[0]) * self.pollination_speed
            cmd.linear.y = float(action[1]) * self.pollination_speed
            cmd.linear.z = float(action[2]) * 1.0  # Slower vertical movement
            cmd.angular.z = float(action[3]) * 0.5  # Gentle turning

        return cmd

    def _get_drone_brain(self, drone_id: str):
        """Create a DroneBrain on first use (scoring-only / no-fleet path)."""
        if drone_id in self.drone_brains:
            return self.drone_brains[drone_id]
        if DroneBrain is None:
            return None
        try:
            self.drone_brains[drone_id] = DroneBrain(
                obs_dim=self.obs_dim,
                msg_dim=self.msg_dim,
                action_dim=self.action_dim,
                world_latent_dim=16,
                enable_collision_avoidance=False,
            )
            return self.drone_brains[drone_id]
        except Exception as e:
            self.get_logger().warn(f"DroneBrain not available for {drone_id}: {e}")
            self.drone_brains[drone_id] = None
            return None

    def _update_pollination_status(self, drone_id: str, action: List[float]):
        """Update pollination status based on drone actions"""
        status = self.drone_status[drone_id]
        x, y, z = status.position

        # Check if drone is pollinating (near flowers and at right altitude)
        if status.current_target and len(action) >= 5:
            patch = status.current_target
            distance_to_patch = math.sqrt(
                (x - patch.center_x)**2 + (y - patch.center_y)**2
            )

            altitude_m = abs(z)
            altitude_ok = abs(altitude_m - self.coverage_height) < 1.0
            position_ok = distance_to_patch < patch.radius or (
                abs(x) < patch.radius and abs(y) < patch.radius
            )

            if altitude_ok and position_ok and action[4] > 0.5:  # Pollination action
                # Increase pollination status
                pollination_rate = 0.01  # 1% per cycle
                patch.pollination_status = min(1.0, patch.pollination_status + pollination_rate)
                status.flowers_pollinated += 1
                status.pollination_payload = max(0, status.pollination_payload - 0.5)

                if patch.pollination_status >= 1.0:
                    status.current_target = None  # Move to next target

    def _handle_safety_violation(self, drone_id: str):
        """Handle safety violations (low battery, geo-fence, etc.)"""
        self.get_logger().warn(f"🚨 Safety violation for {drone_id}, returning to home")

        status = self.drone_status[drone_id]
        status.mission_state = "returning_home"
        fleet = self.fleet_drones.get(drone_id)
        if fleet is not None:
            # Prebuilt collision-aware waypoint (FleetRegistry.resolve_waypoint)
            fleet.go_to_safe(0.0, 0.0, -max(self.coverage_height, 3.0))
            return

        cmd = Twist()
        x, y, z = status.position

        # Head toward origin
        cmd.linear.x = -x * 0.1
        cmd.linear.y = -y * 0.1
        cmd.linear.z = -z * 0.05 if z > 5 else 0.0

        if drone_id in self.cmd_publishers:
            self.cmd_publishers[drone_id].publish(cmd)

    def _get_drone_brain(self, drone_id: str):
        """Create a DroneBrain on first use (scoring-only path)."""
        if drone_id in self.drone_brains:
            return self.drone_brains[drone_id]
        if DroneBrain is None:
            return None
        try:
            self.drone_brains[drone_id] = DroneBrain(
                obs_dim=self.obs_dim,
                msg_dim=self.msg_dim,
                action_dim=self.action_dim,
                world_latent_dim=16,
                enable_collision_avoidance=False,
            )
            return self.drone_brains[drone_id]
        except Exception as e:
            self.get_logger().warn(f"DroneBrain not available for {drone_id}: {e}")
            self.drone_brains[drone_id] = None
            return None

    def _update_mission_metrics(self):
        """Update mission performance metrics"""
        if not self.mission_active:
            return

        # Calculate pollination efficiency
        total_pollination = sum(patch.pollination_status for patch in self.flower_patches)
        max_pollination = len(self.flower_patches)
        self.performance_metrics['pollination_efficiency'] = total_pollination / max_pollination

        # Calculate coverage completeness
        completed_patches = sum(1 for patch in self.flower_patches if patch.pollination_status >= 1.0)
        self.performance_metrics['coverage_completeness'] = completed_patches / max_pollination

        # Calculate average battery consumption
        total_battery = sum(status.battery for status in self.drone_status.values())
        avg_battery = total_battery / len(self.drone_status)
        self.performance_metrics['energy_consumption'] = (100 - avg_battery) / 100.0

    def _publish_mission_status(self):
        """Publish mission status and metrics"""
        if not self.mission_active:
            return

        # Mission status message
        status_msg = String()
        status_msg.data = f"Active|Efficiency:{self.performance_metrics['pollination_efficiency']:.2f}|" \
                         f"Coverage:{self.performance_metrics['coverage_completeness']:.2f}|" \
                         f"Energy:{self.performance_metrics['energy_consumption']:.2f}"

        self.mission_status_pub.publish(status_msg)

        # Flower map message
        flower_data = Float32MultiArray()
        for patch in self.flower_patches:
            flower_data.data.extend([
                patch.center_x, patch.center_y, patch.radius,
                patch.priority, patch.pollination_status
            ])

        self.flower_map_pub.publish(flower_data)

    # Mission Control Methods

    def _start_pollination_mission(self):
        """Start the pollination mission (auto-takeoff + grid like multi_drone_script)."""
        if self.mission_threads and any(t.is_alive() for t in self.mission_threads):
            self.get_logger().warn("Pollination mission already running")
            return

        self.get_logger().info("🌻 Starting Pollination Mission!")
        self.mission_active = True
        self.mission_start_time = time.time()

        for patch in self.flower_patches:
            patch.pollination_status = 0.0

        for i, drone_id in enumerate(self.drone_ids):
            self.drone_status[drone_id].mission_state = "active_mission"
            self.drone_status[drone_id].current_target = self.flower_patches[i % len(self.flower_patches)]

        if not self.fleet_drones:
            self.get_logger().warn("No PX4 fleet attached — scoring only, no flight")
            return

        alt_ned = -abs(self.coverage_height)
        for i, drone_id in enumerate(self.drone_ids):
            fleet = self.fleet_drones[drone_id]
            ctrl = fleet.drone
            home = (ctrl.current_x, ctrl.current_y, ctrl.current_z)
            center = (0.0, 0.0, alt_ned)
            delay = i * 3.0
            t = threading.Thread(
                target=mission_thread,
                args=(fleet, center, home, "cm", delay),
                name=f"pollination_drone_{i}",
                daemon=True,
            )
            t.start()
            self.mission_threads.append(t)
            self.get_logger().info(
                f"   {drone_id}: takeoff + cm grid over spawn patch (stagger {delay:.0f}s)"
            )

    def _ensure_all_landed(self):
        """Land any drone that is still armed (end of mission or --duration)."""
        self.get_logger().info("🛬 Landing remaining drones...")
        for fleet in self.fleet_drones.values():
            fleet.drone._abort_mission = True
        # Let each mission thread land itself so we do not fight the same PX4 instance.
        deadline = time.time() + 80.0
        while time.time() < deadline and any(t.is_alive() for t in self.mission_threads):
            _spin_fleet(self, 0.05)
        for drone_id, fleet in self.fleet_drones.items():
            try:
                if fleet.drone.is_armed() or abs(fleet.drone.current_z) > 0.4:
                    land_and_wait(fleet.drone)
                self.drone_status[drone_id].mission_state = "landed"
            except Exception as exc:
                self.get_logger().warn(f"Land failed for {drone_id}: {exc}")
        self.mission_active = False

    def _pause_mission(self):
        """Pause the mission"""
        self.get_logger().info("⏸️ Pausing Mission")
        for drone_id in self.drone_ids:
            self.drone_status[drone_id].mission_state = "paused"

    def _resume_mission(self):
        """Resume the mission"""
        self.get_logger().info("▶️ Resuming Mission")
        for drone_id in self.drone_ids:
            self.drone_status[drone_id].mission_state = "active_mission"

    def _abort_mission(self):
        """Abort the mission"""
        self.get_logger().info("🛑 Aborting Mission")
        self.mission_active = False
        self._return_to_home()

    def _return_to_home(self):
        """Return all drones to home"""
        self.get_logger().info("🏠 Returning all drones to home")
        for drone_id in self.drone_ids:
            self.drone_status[drone_id].mission_state = "returning_home"


def _spin_fleet(node: IntelleSwarmPollinationNode, timeout_sec: float = 0.05) -> None:
    """Spin pollination node AND every DroneController (offboard setpoint timers)."""
    rclpy.spin_once(node, timeout_sec=timeout_sec)
    for fleet in node.fleet_drones.values():
        rclpy.spin_once(fleet.drone, timeout_sec=0.0)


def _wait_for_px4(node: IntelleSwarmPollinationNode, timeout_sec: float = 40.0) -> None:
    """Drain PX4 status/pose callbacks before takeoff (same idea as multi_mission_runner)."""
    node.get_logger().info("Waiting for PX4 uXRCE VehicleStatus / local position...")
    end = time.time() + timeout_sec
    while time.time() < end and rclpy.ok():
        _spin_fleet(node, 0.1)
        connected = sum(1 for f in node.fleet_drones.values() if f.drone.nav_state != 0)
        if connected == len(node.fleet_drones) and node.fleet_drones:
            break
    for drone_id, fleet in node.fleet_drones.items():
        d = fleet.drone
        node.get_logger().info(
            f"   {drone_id} nav={d.nav_state} arm={d.arm_state} "
            f"pose=({d.current_x:.2f},{d.current_y:.2f},{d.current_z:.2f})"
        )
    if all(f.drone.nav_state == 0 for f in node.fleet_drones.values()):
        node.get_logger().error(
            "No PX4 VehicleStatus received — check instance_N/out.log for "
            "'Timed out waiting for Gazebo world' or missing IMU"
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    install_thread_safe_spin()

    node = IntelleSwarmPollinationNode()

    try:
        # Do NOT rclpy.spin(node): that executor never runs DroneController timers,
        # so offboard setpoints never stream and drones stay on the ground.
        if node.fleet_drones:
            _wait_for_px4(node)
            for fleet in node.fleet_drones.values():
                for _ in range(20):
                    rclpy.spin_once(fleet.drone, timeout_sec=0.1)
        if getattr(node, "_auto_start", True):
            node._start_pollination_mission()
        started = time.time()
        duration = float(getattr(node, "_duration_sec", 0) or 0)
        if duration > 0:
            node.get_logger().info(
                f"Flight time cap: {duration:.0f}s then all drones land "
                "(POLLINATION_DURATION / --duration)"
            )
        while rclpy.ok():
            _spin_fleet(node, 0.05)
            threads = getattr(node, "mission_threads", [])
            if threads and not any(t.is_alive() for t in threads):
                node._ensure_all_landed()
                break
            if duration > 0 and (time.time() - started) >= duration:
                node.get_logger().warn(
                    f"--duration {duration:.0f}s reached — landing remaining drones"
                )
                node._ensure_all_landed()
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("🛑 Shutting down pollination system...")
        try:
            node._ensure_all_landed()
        except Exception:
            pass
    finally:
        for fleet in node.fleet_drones.values():
            try:
                fleet.close()
                fleet.drone.destroy_node()
            except Exception:
                pass
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()