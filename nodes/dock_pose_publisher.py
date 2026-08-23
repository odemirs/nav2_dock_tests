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


class DockPosePublisher(Node):
    """Publishes a noisy dock detection at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("dock_pose_publisher")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "detected_dock_pose")
        self.declare_parameter("dock_x", 2.0)
        self.declare_parameter("dock_y", -0.6)
        self.declare_parameter("dock_yaw", 0.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("noise_stddev_x", 0.0)
        self.declare_parameter("noise_stddev_y", 0.0)
        self.declare_parameter("noise_stddev_yaw", 0.0)
        # 0 draws a nondeterministic seed; any other value makes the sequence repeatable
        self.declare_parameter("noise_seed", 0)

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
        self.timer = self.create_timer(1.0 / rate, self.publish_detection)

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

    def publish_detection(self) -> None:
        """Publish the dock pose with fresh noise drawn for this sample."""
        x = (
            self.dock_x + self.rng.gauss(0.0, self.sigma_x)
            if self.sigma_x
            else self.dock_x
        )
        y = (
            self.dock_y + self.rng.gauss(0.0, self.sigma_y)
            if self.sigma_y
            else self.dock_y
        )
        yaw = (
            self.dock_yaw + self.rng.gauss(0.0, self.sigma_yaw)
            if self.sigma_yaw
            else self.dock_yaw
        )

        msg = PoseStamped()
        # Sim time when use_sim_time is set; getRefinedPose compares this against node_->now()
        msg.header.stamp = self.get_clock().now().to_msg()
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
