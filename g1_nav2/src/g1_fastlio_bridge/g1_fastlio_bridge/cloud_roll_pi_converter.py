#!/usr/bin/env python3
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class CloudRollPiConverter(Node):
    """Republish PointCloud2 after a 180 degree roll around x."""

    def __init__(self):
        super().__init__("g1_mid360_cloud_roll_pi_converter")

        self.input_topic = self.declare_parameter(
            "input_topic", "/utlidar/cloud_livox_mid360"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/g1_fastlio/cloud_livox_mid360_upright"
        ).value
        self.target_frame_id = self.declare_parameter("target_frame_id", "body").value
        self.rotate_roll_pi = _as_bool(
            self.declare_parameter("rotate_roll_pi", True).value
        )

        input_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        output_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, output_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, input_qos
        )
        self._field_offsets = None

        self.get_logger().info(
            f"Converting cloud {self.input_topic} -> {self.output_topic}, "
            f"frame_id={self.target_frame_id}, rotate_roll_pi={self.rotate_roll_pi}"
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        out = PointCloud2()
        out.header = msg.header
        if self.target_frame_id:
            out.header.frame_id = self.target_frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = bytes(msg.data)

        if self.rotate_roll_pi:
            data = bytearray(out.data)
            if self._field_offsets is None:
                offsets = {field.name: field.offset for field in msg.fields}
                self._field_offsets = (
                    offsets.get("x"),
                    offsets.get("y"),
                    offsets.get("z"),
                )
                self.get_logger().info(
                    f"Point field offsets x/y/z={self._field_offsets}"
                )

            x_off, y_off, z_off = self._field_offsets
            if x_off is None or y_off is None or z_off is None:
                self.get_logger().error("PointCloud2 lacks x/y/z fields")
                return

            total = msg.width * msg.height
            step = msg.point_step
            endian = ">" if msg.is_bigendian else "<"
            fmt = endian + "f"
            for i in range(total):
                base = i * step
                y = struct.unpack_from(fmt, data, base + y_off)[0]
                z = struct.unpack_from(fmt, data, base + z_off)[0]
                struct.pack_into(fmt, data, base + y_off, -y)
                struct.pack_into(fmt, data, base + z_off, -z)
            out.data = bytes(data)

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudRollPiConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
