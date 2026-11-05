#!/usr/bin/env python3
"""RoboMaster: search, align, cubic pursuit, return, and CSV logging."""

import csv
import signal
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


def cubic(z0, v0, zf, vf, T):
    """Solve four cubic boundary conditions."""
    A = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [1, T, T*T, T*T*T],
        [0, 1, 2*T, 3*T*T],
    ], dtype=float)
    return np.linalg.solve(A, [z0, v0, zf, vf])


def evaluate(a, t):
    """Return position and velocity."""
    return (
        a[0] + a[1]*t + a[2]*t*t + a[3]*t**3,
        a[1] + 2*a[2]*t + 3*a[3]*t*t,
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Planner(Node):

    def __init__(self):
        super().__init__("planner")

        self.declare_parameter("tag_distance", 1.3)
        self.declare_parameter("standoff", 0.30)
        self.declare_parameter("duration", 13.0)
        self.declare_parameter("auto_return", False)
        self.declare_parameter("yaw_sign", -1.0)

        self.distance = (
            float(self.get_parameter("tag_distance").value)
            - float(self.get_parameter("standoff").value)
        )
        self.T = float(self.get_parameter("duration").value)
        self.auto_return = bool(
            self.get_parameter("auto_return").value
        )
        self.yaw_sign = float(
            self.get_parameter("yaw_sign").value
        )

        if self.distance <= 0 or self.T <= 0:
            raise ValueError("Invalid distance or duration")

        # Zero velocity at both ends; peak speed = 1.5*D/T.
        if 1.5 * self.distance / self.T > 0.16:
            raise ValueError("Trajectory exceeds 0.16 m/s")

        self.outward = cubic(0, 0, self.distance, 0, self.T)

        self.state = "SEARCHING"
        self.bearing = 0.0
        self.visible = False
        self.last_bearing = -1e9
        self.last_direction = 1.0
        self.tag_seen_once = False
        self.stable_since = None
        self.start_time = None
        self.pose = None
        self.pose_time = -1e9
        self.origin = None
        self.turn_accumulated = 0.0
        self.turn_previous_yaw = None
        self.return_distance = None
        self.return_start_x = None
        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(
            String, "/planner/state", 10
        )

        self.create_subscription(
            Float64, "/tag/bearing", self.on_bearing, 10
        )
        self.create_subscription(
            Bool, "/tag/detected", self.on_detection, 10
        )
        self.create_subscription(
            Odometry, "/odom", self.on_odom, 10
        )

        log_dir = Path.home() / "robomaster_ws" / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        filename = time.strftime("mission_%Y%m%d_%H%M%S.csv")
        self.log_path = log_dir / filename
        self.file = self.log_path.open("w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time", "state", "x_ref", "y_ref",
            "x_actual", "y_actual", "yaw_local",
            "bearing_deg", "vx_cmd", "wz_cmd",
        ])

        self.create_timer(0.05, self.control)
        self.get_logger().info(f"Log: {self.log_path}")
        self.get_logger().info("Planner ready: SEARCHING")

    def on_bearing(self, msg):
        if math.isfinite(msg.data):
            self.bearing = float(msg.data)
            self.last_bearing = time.monotonic()
            if abs(self.bearing) > math.radians(5):
                self.last_direction = math.copysign(1, self.bearing)

    def on_detection(self, msg):
        self.visible = bool(msg.data)
        if self.visible:
            self.tag_seen_once = True

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z),
        )
        self.pose = (p.x, p.y, yaw)
        self.pose_time = time.monotonic()

    def local_pose(self):
        if self.origin is None or self.pose is None:
            return 0.0, 0.0, 0.0
        x0, y0, yaw0 = self.origin
        x, y, yaw = self.pose
        dx, dy = x-x0, y-y0
        return (
            dx*math.cos(yaw0) + dy*math.sin(yaw0),
            -dx*math.sin(yaw0) + dy*math.cos(yaw0),
            wrap(yaw-yaw0),
        )

    def start_turn(self, state, yaw):
        self.turn_accumulated = 0.0
        self.turn_previous_yaw = yaw
        self.change_state(state)

    def turn_command(self, yaw):
        delta = wrap(yaw - self.turn_previous_yaw)
        self.turn_accumulated += delta
        self.turn_previous_yaw = yaw

        remaining = math.pi - self.turn_accumulated

        if remaining <= math.radians(3):
            return 0.0, True

        speed = min(0.25, max(0.06, 0.8 * remaining))

        # Sign convention must be verified against odometry.
        return speed, False

    def change_state(self, state):
        if state != self.state:
            self.state = state
            self.get_logger().info(f"State: {state}")

    def control(self):
        now = time.monotonic()
        cmd = Twist()
        xr = yr = 0.0
        x, y, yaw = self.local_pose()

        fresh_odom = now - self.pose_time < 0.5
        fresh_tag = now - self.last_bearing < 0.45

        # Wait for the first odometry message before starting.
        if self.pose is None:
            self.cmd_pub.publish(Twist())
            return

        # Stop safely if odometry is lost after initialization.
        if not fresh_odom and self.state not in ("HOME", "FAULT"):
            self.get_logger().error(
                f"FAULT: odometry stale for {now-self.pose_time:.2f} s"
            )
            self.change_state("FAULT")

        if self.state == "SEARCHING":
            if fresh_tag:
                self.change_state("ALIGNING")
            else:
                search_speed = (
                    0.12 if self.tag_seen_once else 0.30
                )
                cmd.angular.z = (
                    self.yaw_sign * self.last_direction * search_speed
                )

        if self.state == "ALIGNING":
            if not fresh_tag:
                self.stable_since = None
                self.change_state("SEARCHING")
            elif abs(self.bearing) <= math.radians(3):
                if self.stable_since is None:
                    self.stable_since = now
                elif now - self.stable_since >= 0.5:
                    # New local frame: aligned heading becomes +X.
                    self.origin = self.pose
                    x = y = yaw = 0.0
                    self.start_time = now
                    self.change_state("CHASING")
            else:
                self.stable_since = None
                cmd.angular.z = self.yaw_sign * math.copysign(
                    min(1.5*abs(self.bearing), 0.20),
                    self.bearing,
                )

        if self.state == "CHASING":
            t = min(now-self.start_time, self.T)
            xr, vr = evaluate(self.outward, t)

            # Position feedback corrects odometry tracking error.
            cmd.linear.x = float(np.clip(
                vr + 0.5*(xr-x), 0.0, 0.16
            ))

            # Use fresh camera bearing for yaw correction.
            # Brief detection dropouts must not terminate pursuit.
            tag_age = now - self.last_bearing

            if tag_age <= 0.45:
                if abs(self.bearing) > math.radians(5):
                    cmd.angular.z = self.yaw_sign * math.copysign(
                        min(abs(self.bearing), 0.25),
                        self.bearing,
                    )
            elif tag_age <= 2.0:
                # Continue using odometry; no camera correction.
                cmd.angular.z = 0.0
            else:
                self.get_logger().error(
                    f"FAULT: Tag unavailable for {tag_age:.2f} s"
                )
                self.change_state("FAULT")
                cmd = Twist()

            if self.state == "CHASING":
                if x >= self.distance - 0.02:
                    self.change_state("ARRIVED")
                    cmd = Twist()
                    self.start_time = now
                elif t >= self.T and x < self.distance - 0.02:
                    self.change_state("FAULT")
                    cmd = Twist()

        if self.state == "ARRIVED":
            cmd = Twist()
            if now - self.start_time >= 1.0:
                if self.auto_return:
                    # Actual traveled distance, not assumed 1.20 m.
                    self.return_distance = max(0.0, x)
                    self.start_turn("TURN_HOME", yaw)
                else:
                    self.change_state("HOME")

        if self.state == "TURN_HOME":
            cmd.angular.z, finished = self.turn_command(yaw)

            if finished:
                cmd = Twist()
                self.return_start_x = x
                self.start_time = now
                self.change_state("RETURNING")

        if self.state == "RETURNING":
            t = min(now - self.start_time, self.T)

            # Forward travel after rotating 180 degrees.
            travel = max(0.0, self.return_distance)
            a = cubic(0, 0, travel, 0, self.T)
            progress_ref, speed_ref = evaluate(a, t)

            # Original local X decreases while returning.
            progress = self.return_start_x - x
            xr = self.return_start_x - progress_ref

            cmd.linear.x = float(np.clip(
                speed_ref + 0.5*(progress_ref - progress),
                0.0, 0.16
            ))

            # No yaw correction while returning.
            cmd.angular.z = 0.0

            if progress >= travel - 0.02:
                cmd = Twist()
                self.start_turn("TURN_FINAL", yaw)
            elif t >= self.T and progress < travel - 0.02:
                cmd = Twist()
                self.change_state("FAULT")

        if self.state == "TURN_FINAL":
            cmd.angular.z, finished = self.turn_command(yaw)

            if finished:
                cmd = Twist()
                self.change_state("HOME")

        # HOME and FAULT command zero velocity.
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=self.state))

        if now-self.last_log >= 0.05:
            self.writer.writerow([
                f"{now:.3f}", self.state,
                f"{xr:.5f}", f"{yr:.5f}",
                f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}",
                f"{math.degrees(self.bearing):.3f}" if fresh_tag else "",
                f"{cmd.linear.x:.5f}", f"{cmd.angular.z:.5f}",
            ])
            self.file.flush()
            self.last_log = now


def main(args=None):
    # Let Python handle Ctrl+C before rclpy shuts down.
    rclpy.init(
        args=args,
        signal_handler_options=rclpy.signals.SignalHandlerOptions.NO,
    )

    node = Planner()

    def handle_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nCtrl+C received: stopping RoboMaster...")

    except Exception:
        node.get_logger().error("Planner failed; sending stop command")
        raise

    finally:
        if rclpy.ok():
            zero = Twist()
            for _ in range(20):
                node.cmd_pub.publish(zero)
                time.sleep(0.05)

        if not node.file.closed:
            node.file.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
