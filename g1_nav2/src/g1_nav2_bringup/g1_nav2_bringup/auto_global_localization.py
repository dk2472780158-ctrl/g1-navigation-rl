import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Empty


class AutoGlobalLocalization(Node):
    def __init__(self):
        super().__init__("g1_auto_global_localization")
        self.global_service = self.declare_parameter(
            "global_service", "/reinitialize_global_localization"
        ).value
        self.nomotion_service = self.declare_parameter(
            "nomotion_service", "/request_nomotion_update"
        ).value
        self.cmd_vel_input_topic = self.declare_parameter(
            "cmd_vel_input_topic", "/cmd_vel_nav"
        ).value
        self.odom_topic = self.declare_parameter("odom_topic", "/Odometry").value
        self.enable_rotation = bool(
            self.declare_parameter("enable_rotation", True).value
        )
        self.rotation_turns = float(
            self.declare_parameter("rotation_turns", 1.0).value
        )
        self.rotation_velocity = float(
            self.declare_parameter("rotation_velocity", 0.30).value
        )
        self.rotation_timeout = float(
            self.declare_parameter("rotation_timeout", 40.0).value
        )
        self.nomotion_updates = int(
            self.declare_parameter("nomotion_updates", 60).value
        )
        self.update_period = float(
            self.declare_parameter("update_period", 1.0).value
        )
        self.max_position_variance = float(
            self.declare_parameter("max_position_variance", 0.25).value
        )
        self.max_yaw_variance = float(
            self.declare_parameter("max_yaw_variance", 0.50).value
        )
        self.wait_timeout = float(
            self.declare_parameter("wait_timeout", 60.0).value
        )

        self.global_client = self.create_client(Empty, self.global_service)
        self.nomotion_client = self.create_client(Empty, self.nomotion_service)
        self.velocity_pub = self.create_publisher(Twist, self.cmd_vel_input_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 20
        )

        self.started_at = self.get_clock().now()
        self.wait_timer = self.create_timer(0.5, self.wait_for_amcl)
        self.rotation_timer = None
        self.stationary_timer = None
        self.rotation_started_at = None
        self.last_yaw = None
        self.accumulated_rotation = 0.0
        self.update_count = 0
        self.last_covariance = None
        self.finished = False

        self.get_logger().info(
            "Waiting for AMCL. Keep the robot clear and supervised during "
            "active localization."
        )

    def wait_for_amcl(self):
        if self.global_client.service_is_ready():
            self.wait_timer.cancel()
            future = self.global_client.call_async(Empty.Request())
            future.add_done_callback(self.global_localization_done)
            return

        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if elapsed >= self.wait_timeout:
            self.get_logger().error(
                f"AMCL global localization service {self.global_service} was not available"
            )
            self.finish()

    def global_localization_done(self, future):
        try:
            future.result()
        except Exception as error:
            self.get_logger().error(f"Global localization request failed: {error}")
            self.finish()
            return

        self.last_covariance = None
        self.get_logger().info("AMCL global localization started.")
        if self.enable_rotation and self.rotation_turns > 0.0:
            self.start_rotation()
        else:
            self.start_stationary_updates()

    def start_rotation(self):
        self.rotation_started_at = self.get_clock().now()
        self.last_yaw = None
        self.accumulated_rotation = 0.0
        self.rotation_timer = self.create_timer(0.1, self.publish_rotation)
        self.get_logger().info(
            f"Rotating for {self.rotation_turns:.1f} turn(s) at "
            f"{self.rotation_velocity:.2f} rad/s for active localization."
        )

    def publish_rotation(self):
        elapsed = (self.get_clock().now() - self.rotation_started_at).nanoseconds / 1e9
        target_rotation = self.rotation_turns * 2.0 * math.pi
        if self.accumulated_rotation >= target_rotation:
            self.get_logger().info("Active localization rotation completed.")
            self.stop_rotation()
            self.start_stationary_updates()
            return

        if elapsed >= self.rotation_timeout:
            self.get_logger().error(
                "Active localization rotation timed out before completing a full turn."
            )
            self.stop_rotation()
            self.start_stationary_updates()
            return

        command = Twist()
        command.angular.z = self.rotation_velocity
        self.velocity_pub.publish(command)

    def stop_rotation(self):
        self.velocity_pub.publish(Twist())
        if self.rotation_timer is not None:
            self.rotation_timer.cancel()
            self.rotation_timer = None

    def start_stationary_updates(self):
        if self.stationary_timer is None:
            self.stationary_timer = self.create_timer(
                self.update_period, self.request_nomotion_update
            )
            self.get_logger().info("Requesting stationary scan updates.")

    def amcl_pose_callback(self, message):
        covariance = message.pose.covariance
        self.last_covariance = (covariance[0], covariance[7], covariance[35])

    def odom_callback(self, message):
        if self.rotation_timer is None:
            return

        orientation = message.pose.pose.orientation
        yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        if self.last_yaw is not None:
            self.accumulated_rotation += abs(self.normalize_angle(yaw - self.last_yaw))
        self.last_yaw = yaw

    def request_nomotion_update(self):
        if self.is_converged():
            self.get_logger().info(
                "Automatic localization converged after "
                f"{self.update_count} stationary scan updates."
            )
            self.finish()
            return

        if self.update_count >= self.nomotion_updates:
            self.get_logger().error(
                "Automatic localization did not converge. "
                f"Last covariance: {self.format_covariance()}"
            )
            self.finish()
            return

        if not self.nomotion_client.service_is_ready():
            self.get_logger().warn(
                f"AMCL stationary update service {self.nomotion_service} is not available"
            )
            return

        self.update_count += 1
        future = self.nomotion_client.call_async(Empty.Request())
        future.add_done_callback(self.nomotion_update_done)
        if self.update_count % 10 == 0:
            self.get_logger().info(
                f"Stationary update {self.update_count}/{self.nomotion_updates}, "
                f"covariance: {self.format_covariance()}"
            )

    def nomotion_update_done(self, future):
        try:
            future.result()
        except Exception as error:
            self.get_logger().warn(f"AMCL stationary update failed: {error}")

    def is_converged(self):
        if self.last_covariance is None:
            return False
        x_variance, y_variance, yaw_variance = self.last_covariance
        return (
            x_variance <= self.max_position_variance
            and y_variance <= self.max_position_variance
            and yaw_variance <= self.max_yaw_variance
        )

    def format_covariance(self):
        if self.last_covariance is None:
            return "not received"
        x_variance, y_variance, yaw_variance = self.last_covariance
        return (
            f"x={x_variance:.3f}, y={y_variance:.3f}, yaw={yaw_variance:.3f}"
        )

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.stop_rotation()
        self.wait_timer.cancel()
        if self.stationary_timer is not None:
            self.stationary_timer.cancel()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AutoGlobalLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_rotation()
            node.destroy_node()
            rclpy.shutdown()
