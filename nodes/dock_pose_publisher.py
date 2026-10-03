#!/usr/bin/env python3
#
# Copyright (c) 2026 Okan Demir
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish a synthetic dock detection for opennav_docking."""

import math
import random

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from tf_transformations import euler_from_quaternion


class DockPosePublisher(Node):
    """Publishes a noisy dock detection at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("dock_pose_publisher")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "detected_dock_pose")
        self.declare_parameter("dock_x", 1.05)
        self.declare_parameter("dock_y", 2.0)
        self.declare_parameter("dock_yaw", 0.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("noise_stddev_x", 0.0)
        self.declare_parameter("noise_stddev_y", 0.0)
        self.declare_parameter("noise_stddev_yaw", 0.0)
        # 0 draws a nondeterministic seed; any other value makes the sequence repeatable
        self.declare_parameter("noise_seed", 0)
        self.declare_parameter("ground_truth_topic", "")
        self.declare_parameter("ground_truth_timeout", 0.5)

        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        self.dock_x = self.get_parameter("dock_x").value
        self.dock_y = self.get_parameter("dock_y").value
        self.dock_yaw = self.get_parameter("dock_yaw").value
        rate = self.get_parameter("publish_rate").value
        self.sigma_x = self.get_parameter("noise_stddev_x").value
        self.sigma_y = self.get_parameter("noise_stddev_y").value
        self.sigma_yaw = self.get_parameter("noise_stddev_yaw").value
        seed = self.get_parameter("noise_seed").value
        gt_topic = self.get_parameter("ground_truth_topic").value
        self.relative = bool(gt_topic)
        self.gt_timeout = self.get_parameter("ground_truth_timeout").value

        if rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0, got {rate}")
        for name, sigma in (
            ("noise_stddev_x", self.sigma_x),
            ("noise_stddev_y", self.sigma_y),
            ("noise_stddev_yaw", self.sigma_yaw),
        ):
            if sigma < 0.0:
                raise ValueError(f"{name} must be >= 0, got {sigma}")

        self.rng = random.Random(seed if seed != 0 else None)

        # Reliable and shallow: the consumer only ever looks at the newest detection
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(PoseStamped, topic, qos)
        self.ground_truth = None
        if gt_topic:
            self.create_subscription(PoseStamped, gt_topic, self.on_ground_truth, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_detection)

        if gt_topic:
            self.get_logger().info(
                f'Publishing dock detections on "{topic}" at {rate:.1f} Hz in frame '
                f'"{self.frame_id}", relative to the ground-truth robot pose on "{gt_topic}"; '
                f"dock at ({self.dock_x:.3f}, {self.dock_y:.3f}, {self.dock_yaw:.3f} rad) "
                "in its world frame"
            )
        else:
            self.get_logger().info(
                f'Publishing dock detections on "{topic}" at {rate:.1f} Hz in frame '
                f'"{self.frame_id}": pose ({self.dock_x:.3f}, {self.dock_y:.3f}, '
                f"{self.dock_yaw:.3f} rad)"
            )
        if self.sigma_x or self.sigma_y or self.sigma_yaw:
            self.get_logger().info(
                f"Gaussian noise stddev: x={self.sigma_x} m, y={self.sigma_y} m, "
                f'yaw={self.sigma_yaw} rad (seed={seed or "random"})'
            )
        else:
            self.get_logger().info("Noise disabled: publishing ground truth")

    def on_ground_truth(self, msg: PoseStamped) -> None:
        self.ground_truth = msg

    def dock_in_robot_frame(self):
        """Dock pose relative to the ground-truth robot pose, or None without fresh ground truth."""
        gt = self.ground_truth
        if gt is None:
            self.get_logger().warn(
                "No ground-truth robot pose received yet", throttle_duration_sec=5.0)
            return None
        age = (self.get_clock().now() - Time.from_msg(gt.header.stamp)).nanoseconds
        if age * 1e-9 > self.gt_timeout:
            self.get_logger().warn(
                f"Ground-truth robot pose is {age * 1e-9:.2f} s old; not publishing",
                throttle_duration_sec=5.0)
            return None
        rx, ry = gt.pose.position.x, gt.pose.position.y
        q = gt.pose.orientation
        ryaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        dx, dy = self.dock_x - rx, self.dock_y - ry
        c, s = math.cos(ryaw), math.sin(ryaw)
        return (c * dx + s * dy, -s * dx + c * dy, self.dock_yaw - ryaw, gt.header.stamp)

    def publish_detection(self) -> None:
        """Publish the dock pose with fresh noise drawn for this sample."""
        if self.relative:
            rel = self.dock_in_robot_frame()
            if rel is None:
                return
            base_x, base_y, base_yaw, stamp = rel
        else:
            base_x, base_y, base_yaw = self.dock_x, self.dock_y, self.dock_yaw
            stamp = self.get_clock().now().to_msg()

        x = base_x + self.rng.gauss(0.0, self.sigma_x) if self.sigma_x else base_x
        y = base_y + self.rng.gauss(0.0, self.sigma_y) if self.sigma_y else base_y
        yaw = base_yaw + self.rng.gauss(0.0, self.sigma_yaw) if self.sigma_yaw else base_yaw

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
