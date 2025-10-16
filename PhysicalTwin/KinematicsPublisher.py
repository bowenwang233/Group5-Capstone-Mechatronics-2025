#!/usr/bin/env python3
from __future__ import annotations
import argparse
import json
import math
import signal
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

try:
    import zmq
except Exception:
    zmq = None


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def quat_to_yaw(q) -> float:
    """Yaw from geometry_msgs/Quaternion (ENU, Z-up)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    # yaw (psi)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class KinematicsStream(Node):
    """
    No new ROS package—just a script that:
      - Subscribes: /odom_raw (nav_msgs/Odometry), /imu (sensor_msgs/Imu)
      - Computes: velocity v [m/s], acceleration a [m/s^2], steering angle steer [rad]
      - Publishes: ZeroMQ topic -> "telemetry" with JSON: {"t", "v", "a", "steer"}
      - Optional: also prints JSON lines to stdout

    Velocity priority:
      1) odom.twist.twist.linear.x
      2) finite-difference from pose (x,y) if twist.x is unavailable

    Yaw-rate priority:
      1) imu.angular_velocity.z
      2) odom.twist.twist.angular.z
    """

    def __init__(
        self,
        *,
        wheelbase_m: float,
        ema_alpha_v: float,
        ema_alpha_a: float,
        ema_alpha_yaw: float,
        ema_alpha_steer: float,
        max_dt_s: float,
        pub_endpoint: str,
        also_stdout: bool,
        topic_name: str,
        linear_correction: float,
        angular_correction: float,
    ):
        super().__init__("kinematics_stream")

        # Params
        self.L = float(wheelbase_m)
        self.alpha_v = float(ema_alpha_v)
        self.alpha_a = float(ema_alpha_a)
        self.alpha_yaw = float(ema_alpha_yaw)
        self.alpha_steer = float(ema_alpha_steer)
        self.max_dt = float(max_dt_s)
        self.linear_corr = float(linear_correction)
        self.angular_corr = float(angular_correction)
        self.topic_name = topic_name.encode("utf-8")

        # ZeroMQ
        self.pub = None
        self.also_stdout = bool(also_stdout)
        if pub_endpoint and zmq is not None:
            ctx = zmq.Context.instance()
            self.pub = ctx.socket(zmq.PUB)
            self.pub.bind(pub_endpoint)
            self.get_logger().info(f"ZeroMQ PUB bound at {pub_endpoint}")
        elif pub_endpoint and zmq is None:
            self.get_logger().warn("pyzmq not installed; will only print to stdout if enabled.")

        # Subscriptions (sensor-style QoS)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)

        # State
        self._last_t: Optional[float] = None  # sec
        self._last_v: Optional[float] = None  # m/s
        self._last_x: Optional[float] = None
        self._last_y: Optional[float] = None

        self._v_f: Optional[float] = None
        self._a_f: Optional[float] = None
        self._yaw_rate_f: Optional[float] = None
        self._steer_f: Optional[float] = None

        self._last_pub_t: Optional[float] = None  # for consistent t in payload

        self.get_logger().info("kinematics_stream ready (listening to /odom_raw and /imu)")

    # -------- helpers --------
    @staticmethod
    def _ema(prev: Optional[float], x: float, alpha: float) -> float:
        return x if prev is None else alpha * x + (1.0 - alpha) * prev

    # -------- callbacks --------
    def _on_imu(self, msg: Imu):
        t = stamp_to_sec(msg.header.stamp)
        yaw_rate = float(msg.angular_velocity.z) * self.angular_corr  # rad/s
        self._yaw_rate_f = self._ema(self._yaw_rate_f, yaw_rate, self.alpha_yaw)
        # try to compute steer if we already have v
        self._maybe_update_steer()
        # publish using the latest known v/a/steer
        self._publish(t)

    def _on_odom(self, msg: Odometry):
        t = stamp_to_sec(msg.header.stamp)

        # --- Velocity source #1: provided by odom twist ---
        v_odom = float(msg.twist.twist.linear.x) if msg.twist and msg.twist.twist else None

        # --- Fallback Velocity source #2: finite diff of pose ---
        v_fd = None
        if msg.pose and msg.pose.pose and msg.pose.pose.position:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            if self._last_t is not None and self._last_x is not None and self._last_y is not None:
                dt_fd = t - self._last_t
                if dt_fd > 0.0 and dt_fd <= self.max_dt:
                    dx = x - self._last_x
                    dy = y - self._last_y
                    v_fd = math.hypot(dx, dy) / dt_fd
            self._last_x, self._last_y = x, y

        # Choose velocity
        v_raw = v_odom if v_odom is not None else v_fd
        if v_raw is None:
            # nothing to do yet; just advance time
            self._last_t = t
            return

        # Apply linear correction (tutorial-style scale)
        v_raw *= self.linear_corr

        # Smooth velocity
        self._v_f = self._ema(self._v_f, v_raw, self.alpha_v)

        # Derive acceleration when possible
        if self._last_t is not None and self._last_v is not None:
            dt = t - self._last_t
            if dt > 0.0:
                a_raw = (self._v_f - self._last_v) / dt
                self._a_f = self._ema(self._a_f, a_raw, self.alpha_a)

        # Update lasts (for next derivative step)
        self._last_t = t
        self._last_v = self._v_f

        # If we don't have IMU yet, we can still use odom angular z as yaw rate
        if self._yaw_rate_f is None:
            yaw_rate_odom = float(msg.twist.twist.angular.z) if msg.twist and msg.twist.twist else None
            if yaw_rate_odom is not None:
                self._yaw_rate_f = self._ema(self._yaw_rate_f, yaw_rate_odom * self.angular_corr, self.alpha_yaw)

        # Update steering from whatever yaw_rate we have
        self._maybe_update_steer()

        # Publish
        self._publish(t)

    # -------- core math --------
    def _maybe_update_steer(self):
        if self._yaw_rate_f is None or self._v_f is None:
            return
        v_eff = max(self._v_f, 1e-3)  # avoid division near zero
        steer_raw = math.atan(self.L * self._yaw_rate_f / v_eff)
        self._steer_f = self._ema(self._steer_f, steer_raw, self.alpha_steer)

    # -------- output --------
    def _publish(self, t_now: float):
        # Use the freshest timestamp we have seen
        self._last_pub_t = t_now

        payload = {"t": float(self._last_pub_t)}
        if self._v_f is not None:
            payload["v"] = float(self._v_f)
        if self._a_f is not None:
            payload["a"] = float(self._a_f)
        if self._steer_f is not None:
            payload["steer"] = float(self._steer_f)

        if len(payload) <= 1:
            return  # nothing to send yet

        if self.pub is not None:
            self.pub.send_multipart([self.topic_name, json.dumps(payload).encode("utf-8")])

        if self.also_stdout:
            print(json.dumps(payload), flush=True)


def main():
    ap = argparse.ArgumentParser(description="Compute v/a/steer from /odom_raw + /imu and stream via ZeroMQ.")
    ap.add_argument("--wheelbase-m", type=float, default=0.26, help="Vehicle wheelbase in meters.")
    ap.add_argument("--ema-alpha-v", type=float, default=0.35, help="EMA smoothing for velocity.")
    ap.add_argument("--ema-alpha-a", type=float, default=0.35, help="EMA smoothing for acceleration.")
    ap.add_argument("--ema-alpha-yaw", type=float, default=0.35, help="EMA smoothing for yaw-rate.")
    ap.add_argument("--ema-alpha-steer", type=float, default=0.35, help="EMA smoothing for steering angle.")
    ap.add_argument("--max-dt-s", type=float, default=0.25, help="Reset derivative if sample gap exceeds this (s).")

    ap.add_argument("--linear-correction", type=float, default=1.0,
                    help="Scale factor applied to velocity (tutorial correction).")
    ap.add_argument("--angular-correction", type=float, default=1.0,
                    help="Scale factor applied to yaw-rate (tutorial correction).")

    ap.add_argument("--pub-endpoint", type=str, default="tcp://*:5557",
                    help='ZeroMQ PUB endpoint to bind (e.g., "tcp://*:5557"). Empty disables ZMQ.')
    ap.add_argument("--topic", type=str, default="telemetry", help="ZeroMQ topic string.")
    ap.add_argument("--stdout", action="store_true", help="Also print JSON lines to stdout.")
    args = ap.parse_args()

    # Graceful Ctrl+C
    def _sigint(_sig, _frm):
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)

    rclpy.init()

    node = KinematicsStream(
        wheelbase_m=args.wheelbase_m,
        ema_alpha_v=args.ema_alpha_v,
        ema_alpha_a=args.ema_alpha_a,
        ema_alpha_yaw=args.ema_alpha_yaw,
        ema_alpha_steer=args.ema_alpha_steer,
        max_dt_s=args.max_dt_s,
        pub_endpoint=args.pub_endpoint,
        also_stdout=args.stdout,
        topic_name=args.topic,
        linear_correction=args.linear_correction,
        angular_correction=args.angular_correction,
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()