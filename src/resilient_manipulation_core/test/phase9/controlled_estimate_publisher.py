#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ControlledEstimatePublisher(Node):

    def __init__(self, mode: str):
        super().__init__("controlled_estimate_publisher")

        self.mode = mode

        self.publisher = self.create_publisher(
            PoseStamped,
            "/object_pose_estimate",
            10,
        )

        self.get_logger().info(
            f"Controlled estimate publisher started | mode={self.mode}"
        )


    def make_pose(self, stamp):
        msg = PoseStamped()

        msg.header.frame_id = "world"
        msg.header.stamp = stamp

        msg.pose.position.x = 0.387
        msg.pose.position.y = 0.050
        msg.pose.position.z = 0.470

        msg.pose.orientation.w = 1.0

        return msg


    def make_stale_pose(self):
        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        stale_ns = (
            now_ns
            - int(5.0e9)
        )

        stale_stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        stale_stamp.sec = int(
            stale_ns // 1_000_000_000
        )

        stale_stamp.nanosec = int(
            stale_ns % 1_000_000_000
        )

        return self.make_pose(
            stale_stamp
        )


    def wait_for_mission_subscriber(self):
        start_time = time.monotonic()

        while (
            time.monotonic() - start_time
            < 15.0
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if (
                self.publisher.get_subscription_count()
                > 0
            ):
                self.get_logger().info(
                    "Mission subscriber detected."
                )

                return True

        self.get_logger().error(
            "Timed out waiting for mission subscriber."
        )

        return False


    def publish_fixed_stale_samples(
        self,
        stale_pose,
        duration_seconds: float,
    ):
        start_time = time.monotonic()

        while (
            time.monotonic() - start_time
            < duration_seconds
        ):
            # Important:
            # same PoseStamped timestamp is reused.
            # Therefore these are NOT new recovery candidates.
            self.publisher.publish(
                stale_pose
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            time.sleep(0.05)

    def run_fresh(self):
        self.get_logger().info(
            "Publishing controlled fresh estimate."
        )

        start_time = time.monotonic()

        while (
            time.monotonic() - start_time
            < 5.0
        ):
            fresh_pose = self.make_pose(
                self.get_clock()
                .now()
                .to_msg()
            )

            self.publisher.publish(
                fresh_pose
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            time.sleep(0.05)


        self.get_logger().info(
            "Controlled fresh-estimate sequence complete."
        )

        return 0

    def run_stale_to_fresh(self):
        stale_pose = self.make_stale_pose()

        self.get_logger().info(
            "Publishing fixed stale estimate continuously for 1.5 s."
        )

        self.publish_fixed_stale_samples(
            stale_pose,
            1.5,
        )


        fresh_pose = self.make_pose(
            self.get_clock()
            .now()
            .to_msg()
        )


        self.get_logger().info(
            "Publishing NEW fresh estimate."
        )


        for _ in range(5):
            self.publisher.publish(
                fresh_pose
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            time.sleep(0.05)


        self.get_logger().info(
            "P9-S02 stale-to-fresh sequence complete."
        )

        return 0
    def run_persistent_stale(self):
        stale_pose = self.make_stale_pose()


        self.get_logger().info(
            "Publishing persistent stale estimate "
            "with one fixed timestamp."
        )


        self.publish_fixed_stale_samples(
            stale_pose,
            4.5,
        )


        self.get_logger().info(
            "P9-S03 persistent-stale sequence complete."
        )

        return 0


    def run_scenario(self):
        if not self.wait_for_mission_subscriber():
            return 1
        if self.mode == "fresh":
            return self.run_fresh()

        if self.mode == "stale-to-fresh":
            return self.run_stale_to_fresh()


        if self.mode == "persistent-stale":
            return self.run_persistent_stale()


        self.get_logger().error(
            f"Unsupported mode: {self.mode}"
        )

        return 2


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Controlled estimate publisher for "
            "Phase-9 automated recovery scenarios."
        )
    )

    parser.add_argument(
        "--mode",
        choices=[
            "fresh",
            "stale-to-fresh",
            "persistent-stale",
        ],
        default="stale-to-fresh",
    )

    return parser.parse_args()


def main():
    arguments = parse_arguments()

    rclpy.init()

    node = ControlledEstimatePublisher(
        arguments.mode
    )

    try:
        result = node.run_scenario()

    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(
        result
    )


if __name__ == "__main__":
    main()
