#!/usr/bin/env python3
"""AprilTag detector: live video, centerline, IDs and bearing."""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float64, Int32


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__("apriltag_detector")

        self.declare_parameter("target_id", 3)
        self.declare_parameter("horizontal_fov_deg", 80.0)
        self.declare_parameter("scale", 2.0)

        self.target_id = int(
            self.get_parameter("target_id").value
        )
        self.fov = math.radians(
            float(self.get_parameter("horizontal_fov_deg").value)
        )
        self.scale = float(self.get_parameter("scale").value)

        self.fx = None
        self.principal_x = None

        families = [
            ("36h11", cv2.aruco.DICT_APRILTAG_36h11),
            ("25h9", cv2.aruco.DICT_APRILTAG_25h9),
            ("16h5", cv2.aruco.DICT_APRILTAG_16h5),
            ("36h10", cv2.aruco.DICT_APRILTAG_36h10),
        ]

        self.dictionaries = [
            (name, cv2.aruco.Dictionary_get(code))
            for name, code in families
        ]

        self.parameters = cv2.aruco.DetectorParameters_create()

        # Improve small-marker detection
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 31
        self.parameters.adaptiveThreshWinSizeStep = 4
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )

        self.bearing_pub = self.create_publisher(
            Float64, "/tag/bearing", 10
        )
        self.id_pub = self.create_publisher(
            Int32, "/tag/id", 10
        )
        self.detected_pub = self.create_publisher(
            Bool, "/tag/detected", 10
        )

        self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            Image,
            "/camera/image_color",
            self.image_callback,
            10,
        )

        cv2.namedWindow(
            "RoboMaster AprilTag Detector",
            cv2.WINDOW_NORMAL
        )

        self.get_logger().info(
            "Detector ready | Target Tag %d" % self.target_id
        )

    def camera_info_callback(self, msg):
        if msg.k[0] > 0:
            self.fx = float(msg.k[0])
            self.principal_x = float(msg.k[2])

    def image_callback(self, msg):
        try:
            if msg.encoding not in ("rgb8", "bgr8"):
                return

            raw = np.frombuffer(msg.data, dtype=np.uint8)
            image = raw.reshape(msg.height, msg.step)
            image = image[:, :msg.width * 3]
            image = image.reshape(
                msg.height, msg.width, 3
            ).copy()

            if msg.encoding == "rgb8":
                image = cv2.cvtColor(
                    image, cv2.COLOR_RGB2BGR
                )

            height, width = image.shape[:2]
            center_x = width // 2
            center_y = height // 2

            if self.fx is not None:
                fx = self.fx
                cx = self.principal_x
            else:
                fx = width / (2 * math.tan(self.fov / 2))
                cx = width / 2

            gray = cv2.cvtColor(
                image, cv2.COLOR_BGR2GRAY
            )

            scaled = cv2.resize(
                gray,
                None,
                fx=self.scale,
                fy=self.scale,
                interpolation=cv2.INTER_CUBIC,
            )

            detections = []

            for family, dictionary in self.dictionaries:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    scaled,
                    dictionary,
                    parameters=self.parameters,
                )

                if ids is None:
                    continue

                for i, tag_id in enumerate(ids.flatten()):
                    pts = corners[i][0] / self.scale
                    detections.append(
                        (family, int(tag_id), pts)
                    )

            # Vertical alignment line
            cv2.line(
                image,
                (center_x, 0),
                (center_x, height),
                (0, 255, 0),
                2,
            )

            # Small horizontal center reference
            cv2.line(
                image,
                (center_x - 15, center_y),
                (center_x + 15, center_y),
                (0, 255, 0),
                2,
            )

            target_found = False
            tag_rows = []

            for family, tag_id, pts in detections:
                pts_int = pts.astype(np.int32)

                tag_x = float(np.mean(pts[:, 0]))
                tag_y = float(np.mean(pts[:, 1]))

                dx = tag_x - cx
                bearing = math.atan2(dx, fx)
                bearing_deg = math.degrees(bearing)

                if bearing_deg > 1:
                    direction = "RIGHT"
                elif bearing_deg < -1:
                    direction = "LEFT"
                else:
                    direction = "CENTER"

                is_target = tag_id == self.target_id

                tag_rows.append(
                    (tag_id, family, bearing_deg, direction, is_target)
                )

                color = (
                    (0, 255, 0)
                    if is_target
                    else (0, 200, 255)
                )

                cv2.polylines(
                    image,
                    [pts_int],
                    True,
                    color,
                    2,
                )

                tx = int(tag_x)
                ty = int(tag_y)

                cv2.circle(
                    image,
                    (tx, ty),
                    4,
                    color,
                    -1,
                )

                cv2.line(
                    image,
                    (center_x, ty),
                    (tx, ty),
                    color,
                    2,
                )

                # Compact tag ID near marker
                cv2.putText(
                    image,
                    f"T{tag_id}",
                    (tx - 12, max(15, int(np.min(pts[:, 1])) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )

                if is_target and not target_found:
                    target_found = True

                    self.bearing_pub.publish(
                        Float64(data=float(bearing))
                    )
                    self.id_pub.publish(
                        Int32(data=tag_id)
                    )

            self.detected_pub.publish(
                Bool(data=target_found)
            )

            if target_found:
                status = f"TARGET {self.target_id}: FOUND"
                status_color = (0, 255, 0)
            else:
                status = f"TARGET {self.target_id}: NOT FOUND"
                status_color = (0, 0, 255)

            # Compact dashboard, separate from the tag markers
            tag_rows.sort(key=lambda row: row[0])

            panel_height = min(height, 72 + 25 * len(tag_rows))
            panel_width = min(width, 380)

            overlay = image.copy()
            cv2.rectangle(
                overlay,
                (0, 0),
                (panel_width, panel_height),
                (20, 20, 20),
                -1,
            )
            cv2.addWeighted(
                overlay, 0.75, image, 0.25, 0, image
            )

            cv2.putText(
                image,
                f"TARGET T{self.target_id}: "
                f"{'FOUND' if target_found else 'NOT FOUND'}",
                (10, 23),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0) if target_found else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                image,
                "TAG     BEARING     DIRECTION",
                (10, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )

            for index, (tid, family, angle, direction, selected) in enumerate(tag_rows):
                y = 72 + index * 25
                if y >= height:
                    break

                row_color = (0, 255, 0) if selected else (0, 200, 255)

                cv2.putText(
                    image,
                    f"T{tid:<5} {angle:+7.1f} deg   {direction}",
                    (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    row_color,
                    1,
                    cv2.LINE_AA,
                )

            cv2.imshow(
                "RoboMaster AprilTag Detector",
                image
            )
            cv2.waitKey(1)

        except Exception as exc:
            self.get_logger().error(str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
