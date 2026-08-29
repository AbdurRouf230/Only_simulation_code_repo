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
import yaml
import time
from typing import Dict, Any, List, Tuple, Optional
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS message types
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, NavSatFix
from std_msgs.msg import Float32, Bool, String, Header
from geographic_msgs.msg import GeoPoseStamped

# Custom message types (would be defined separately)
from std_msgs.msg import Float32MultiArray  # Temporary for flower detection data

# IntelleSwarm core imports
import sys
sys.path.insert(0, '/Users/zrahman/intelleswarm-ai')

try:
    from genai_framework.sdk import DroneBrain, EnvReading, GeoFence, SafetyManager, Pose
    from assistive_pollination.swarm_coordination import SwarmCoordinator
    from assistive_pollination.agricultural_ai import FlowerDetector, CoverageOptimizer
    from assistive_pollination.mission_planning import PollinationMissionPlanner
except ImportError as e:
    print(f"Warning: Could not import IntelleSwarm modules: {e}")
    print("Running in simulation mode without full AI integration")

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
        self.drone_ids = [f"drone_{i}" for i in range(1, 7)]  # 6 drones
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
            # Flower detection system
            self.flower_detector = FlowerDetector()

            # Coverage optimization
            self.coverage_optimizer = CoverageOptimizer()

            self.get_logger().info("✅ Agricultural AI components loaded")
        except Exception as e:
            self.get_logger().warn(f"Agricultural AI components not available: {e}")
            self.flower_detector = None
            self.coverage_optimizer = None

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

        # Initialize drone brains with pollination-specific parameters
        self.drone_brains = {}
        self.drone_status = {}

        for drone_id in self.drone_ids:
            try:
                self.drone_brains[drone_id] = DroneBrain(
                    obs_dim=self.obs_dim,
                    msg_dim=self.msg_dim,
                    action_dim=self.action_dim,
                    world_latent_dim=16,
                    enable_collision_avoidance=True
                )
            except Exception as e:
                self.get_logger().warn(f"DroneBrain not available for {drone_id}: {e}")

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

        # Create geo-fence from config
        fence_vertices = self.config['safety']['geo_fence']['vertices']
        self.farm_fence = GeoFence(vertices=fence_vertices)

        try:
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

    def _image_callback(self, drone_id: str, msg: Image):
        """Process camera images for flower detection"""
        if not self.flower_detector:
            return

        try:
            # Convert ROS image to format for flower detection
            # This would normally involve cv_bridge conversion
            # For simulation, we'll mock the detection

            status = self.drone_status[drone_id]
            x, y, z = status.position

            # Mock flower detection based on proximity to known patches
            detected_flowers = []
            for patch in self.flower_patches:
                distance = math.sqrt((x - patch.center_x)**2 + (y - patch.center_y)**2)
                if distance < patch.radius + 10:  # Detection range
                    detected_flowers.append({
                        'type': patch.flower_type,
                        'distance': distance,
                        'bearing': math.atan2(patch.center_y - y, patch.center_x - x),
                        'priority': patch.priority
                    })

            # Update drone's target if needed
            if detected_flowers and not status.current_target:
                # Find highest priority flower
                best_flower = max(detected_flowers, key=lambda f: f['priority'])
                # Find corresponding patch
                for patch in self.flower_patches:
                    if patch.flower_type == best_flower['type']:
                        dx = x - patch.center_x
                        dy = y - patch.center_y
                        if abs(dx) < patch.radius and abs(dy) < patch.radius:
                            status.current_target = patch
                            break

        except Exception as e:
            self.get_logger().warn(f"Flower detection error for {drone_id}: {e}")

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
        if not self.mission_active:
            return

        current_time = time.time()

        # Process each drone
        for drone_id in self.drone_ids:
            if drone_id not in self.drone_status:
                continue

            status = self.drone_status[drone_id]

            # Safety check
            if self._check_safety(drone_id):
                self._execute_drone_mission(drone_id)
            else:
                self._handle_safety_violation(drone_id)

        # Update mission metrics
        self._update_mission_metrics()

        # Publish mission status
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
        brain = self.drone_brains.get(drone_id)

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

            altitude_ok = abs(z - self.coverage_height) < 1.0
            position_ok = distance_to_patch < patch.radius

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

        # Simple RTH command
        cmd = Twist()
        x, y, z = status.position

        # Head toward origin
        cmd.linear.x = -x * 0.1
        cmd.linear.y = -y * 0.1
        cmd.linear.z = -z * 0.05 if z > 5 else 0.0

        if drone_id in self.cmd_publishers:
            self.cmd_publishers[drone_id].publish(cmd)

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
        """Start the pollination mission"""
        self.get_logger().info("🌻 Starting Pollination Mission!")
        self.mission_active = True
        self.mission_start_time = time.time()

        # Reset flower patch status
        for patch in self.flower_patches:
            patch.pollination_status = 0.0

        # Set all drones to mission state
        for drone_id in self.drone_ids:
            self.drone_status[drone_id].mission_state = "active_mission"

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


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    node = IntelleSwarmPollinationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down pollination system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()