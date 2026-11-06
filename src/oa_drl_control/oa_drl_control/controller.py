#!/usr/bin/env python3
"""
Test-time controller for DRL-based collision avoidance.

Loads a trained Q-network and drives the robot with a greedy policy:
at every pre-processed LiDAR scan it picks the action with the highest
Q-value and maps it to an angular velocity (paper §3.4). Collisions
trigger a respawn, and every step is logged to CSV for evaluation.
"""

import csv
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Empty, Trigger

os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
import tensorflow as tf  # noqa: E402  (must follow the protobuf env var)

# Allow GPU memory growth to avoid OOM crashes
gpus = tf.config.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError as e:
        print(e)

DEFAULT_WS = Path.home() / 'ros_ws'


class Controller(Node):
    """Greedy policy runner for a trained DDQN obstacle-avoidance model."""

    def __init__(self):
        super().__init__('controller')

        # Node parameters
        self.declare_parameter('collision_tol', 0.2)     # metres
        self.declare_parameter('linear_velocity', 0.2)   # constant forward speed, m/s
        self.declare_parameter('lidar_max_range', 5.0)
        self.declare_parameter('max_steps', 19800)       # test length, one step per LiDAR scan
        self.declare_parameter('model_path', str(DEFAULT_WS / 'models' / 'best_model.keras'))
        self.declare_parameter('log_dir', str(DEFAULT_WS / 'logs'))

        self.lidar_max_range = self.get_parameter('lidar_max_range').value
        self.collision_tol = self.get_parameter('collision_tol').value / self.lidar_max_range
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.max_steps = self.get_parameter('max_steps').value
        model_path = os.path.expanduser(self.get_parameter('model_path').value)
        log_dir = os.path.expanduser(self.get_parameter('log_dir').value)

        # Subscribers
        self.scan_subscription = self.create_subscription(
            Float32MultiArray,
            '/lidar_data',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.min_dist_publisher = self.create_publisher(Float32, '/min_lidar_distance', 10)

        # Service clients
        self.respawn_client = self.create_client(Trigger, '/randomize_robot_pose')
        self.pause_physics_client = self.create_client(Empty, '/pause_physics')
        self.unpause_physics_client = self.create_client(Empty, '/unpause_physics')

        # Metrics and state
        self.step_count = 0
        self.collision_count = 0
        self.feedback_rate = 2000
        self.test_finished = False

        # CSV logging
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'test_{timestamp}.csv')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['step', 'min_distance_m', 'action', 'omega'])
        self.get_logger().info(f'Logging CSV to: {self.csv_path}')

        # Load trained model
        self.get_logger().info(f'Loading model: {model_path}')
        self.model = tf.keras.models.load_model(model_path, compile=False)

        self.navigation_active = True
        self.stop_flag = False
        self.is_resetting = False
        self.skip_lidar_scans = 0
        self.state = None

        self.get_logger().info(
            f'Controller initialised in TEST mode: max_steps={self.max_steps}'
        )

    def scan_callback(self, msg: Float32MultiArray):
        """Receive pre-processed LiDAR data and trigger the control loop."""
        if self.skip_lidar_scans > 0:
            self.skip_lidar_scans -= 1
            return
        if self.is_resetting:
            return

        self.state = np.array(msg.data) / self.lidar_max_range
        self.state = self.state.reshape(1, len(self.state))
        self.control_loop_callback()

    def check_collision(self, distances) -> bool:
        """Return True if the nearest normalised range is below the collision threshold."""
        if distances.size == 0:
            return False

        min_range = np.min(distances)
        if min_range < self.collision_tol:
            self.get_logger().warn(
                f'Collision detected! Min range: {min_range * self.lidar_max_range:.3f} m')
            self.stop_flag = True
            return True

        return False

    def respawn_robot(self):
        """Pause physics, then call /randomize_robot_pose to respawn the robot."""
        self.is_resetting = True
        self.pause_physics_client.call_async(Empty.Request())

        if not self.respawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /randomize_robot_pose not available!')
            self.is_resetting = False
            return

        future = self.respawn_client.call_async(Trigger.Request())
        future.add_done_callback(self._respawn_done_callback)

    def _respawn_done_callback(self, future):
        """Resume navigation once the respawn service call completes."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'Respawn succeeded: {result.message}')
            else:
                self.get_logger().warn(f'Respawn failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Respawn service call error: {e}')

        self.stop_flag = False
        self.state = None
        self.skip_lidar_scans = 15  # discard stale LiDAR data after respawn
        self.is_resetting = False
        self.unpause_physics_client.call_async(Empty.Request())

    def finish_test(self):
        """Stop the test and log collision results."""
        self.stop_robot()
        self.test_finished = True
        self.navigation_active = False
        self.csv_file.close()
        self.get_logger().info('=' * 60)
        self.get_logger().info('TEST COMPLETED')
        self.get_logger().info(f'Total steps: {self.step_count}')
        self.get_logger().info(f'Total collisions: {self.collision_count}')
        self.get_logger().info(f'CSV saved to: {self.csv_path}')
        self.get_logger().info('=' * 60)

    def control_loop_callback(self):
        """One greedy control step, invoked once per LiDAR scan."""
        if self.state is None or self.test_finished or not self.navigation_active:
            return

        # Stop once the test duration has been reached
        if self.step_count >= self.max_steps:
            self.finish_test()
            return

        # 1. Collision check
        if self.check_collision(self.state) or self.stop_flag:
            self.stop_robot()
            self.collision_count += 1
            self.get_logger().warn(
                f'Collision #{self.collision_count} at step {self.step_count}. Respawning...')
            self.respawn_robot()
            return

        # 2. Greedy policy: index m in [0, 10] of the max Q-value
        q_values = self.model(self.state, training=False).numpy()
        m = np.argmax(q_values[0])

        # 3. Angular velocity from the discrete action (paper §3.4)
        omega_m = -0.8 + 0.16 * m

        # 4. Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = self.linear_velocity
        cmd_msg.angular.z = float(omega_m)
        self.cmd_vel_publisher.publish(cmd_msg)

        min_distance_meters = float(np.min(self.state) * self.lidar_max_range)
        msg = Float32()
        msg.data = min_distance_meters
        self.min_dist_publisher.publish(msg)

        # 5. CSV logging and periodic feedback
        self.csv_writer.writerow(
            [self.step_count, f'{min_distance_meters:.4f}', m, f'{omega_m:.2f}'])
        if self.step_count % self.feedback_rate == 0:
            self.csv_file.flush()
            self.get_logger().info(f'Step {self.step_count}')

        self.step_count += 1

    def stop_robot(self):
        """Publish a zero-velocity command to halt the robot."""
        self.cmd_vel_publisher.publish(Twist())
        self.get_logger().info('Robot stopped')


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
