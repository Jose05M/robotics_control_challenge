#!/usr/bin/env python3
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from control_msgs.msg import JointJog

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class PerturbationGenerator(Node):

    def __init__(self):

        super().__init__("xarm_perturbation_injector")

        self.output_topic = self.declare_parameter(
            "output_topic", "/servo_server/delta_joint_cmds"
        ).value

        self.enabled = bool(self.declare_parameter("enabled", True).value)

        self.mode = str(
            self.declare_parameter("mode", "sine").value
        ).lower().strip()

        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.02).value
        )

        self.max_joint = float(
            self.declare_parameter("max_joint_speed", 0.5).value
        )

        # sine
        self.sine_freq_hz = float(
            self.declare_parameter("sine_freq_hz", 8.0).value
        )

        self.sine_amp_joint = float(
            self.declare_parameter("sine_amp_joint", 0.1).value
        )

        self.sine_axis = int(
            self.declare_parameter("sine_axis", 1).value
        )  # joint index

        # gaussian
        self.noise_std_joint = float(
            self.declare_parameter("noise_std_joint", 0.05).value
        )

        base = self.declare_parameter(
            "base_joint", [0,0,0,0,0,0]
        ).value

        try:
            self.base_joint = np.array(base, dtype=float)
        except:
            self.base_joint = np.zeros(6)

        # debug
        self.debug = bool(self.declare_parameter("debug", True).value)
        self.debug_period_s = float(
            self.declare_parameter("debug_period_s", 1.0).value
        )

        self._out_count = 0
        self._last_dbg_wall = time.time()

        qos_name = str(
            self.declare_parameter("pub_reliability", "reliable").value
        ).lower().strip()

        reliability = (
            ReliabilityPolicy.RELIABLE
            if qos_name in ("reliable","r")
            else ReliabilityPolicy.BEST_EFFORT
        )

        self.pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(
            JointJog,
            self.output_topic,
            self.pub_qos
        )

        self.t0 = time.time()

        self.rng = np.random.default_rng(7)

        self.timer = self.create_timer(
            self.publish_period_s,
            self.tick
        )

        self.get_logger().info(
            "✅ xarm_joint_perturbation_injector\n"
            f"OUT: {self.output_topic}\n"
            f"mode={self.mode}"
        )


    def _dp(self):

        if self.mode == "off":
            return np.zeros(6)

        if self.mode == "gaussian":
            return self.rng.normal(
                0.0,
                self.noise_std_joint,
                size=6
            )

        # sine
        s = math.sin(
            2.0 * math.pi *
            self.sine_freq_hz *
            (time.time() - self.t0)
        )

        dp = np.zeros(6)

        axis = int(self.sine_axis)

        dp[axis] = self.sine_amp_joint * s

        return dp


    def tick(self):

        if not self.enabled:
            v = np.zeros(6)
            self._publish(v,"DISABLED")
            return

        v = np.clip(
            self.base_joint + self._dp(),
            -self.max_joint,
            self.max_joint
        )

        self._publish(v,"RUN")


    def _publish(self,v,note=""):

        msg = JointJog()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "link_base"

        msg.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

        msg.velocities = v.tolist()

        msg.duration = 0.02

        self.pub.publish(msg)

        self._out_count += 1

        if self.debug:

            now = time.time()

            if now - self._last_dbg_wall >= self.debug_period_s:

                self.get_logger().info(
                    f"[dbg] out={self._out_count} {note} qdot={np.round(v,3)}"
                )

                self._last_dbg_wall = now


def main():

    rclpy.init()

    n = PerturbationGenerator()

    rclpy.spin(n)

    n.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()