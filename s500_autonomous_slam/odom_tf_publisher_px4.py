#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class OdomTfPublisherPx4(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher_px4')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Odom TF Publisher PX4 Initialized.')

    def euler_from_quaternion(self, w, x, y, z):
        # Roll
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        # Yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return w, x, y, z

    def odom_callback(self, msg):
        now = self.get_clock().now().to_msg()
        
        # Position NED to ENU
        x_enu = msg.position[1]
        y_enu = msg.position[0]
        z_enu = -msg.position[2]
        
        # Velocity NED to ENU
        vx_enu = msg.velocity[1]
        vy_enu = msg.velocity[0]
        vz_enu = -msg.velocity[2]

        # Orientation NED to ENU
        q_ned_w, q_ned_x, q_ned_y, q_ned_z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        roll_ned, pitch_ned, yaw_ned = self.euler_from_quaternion(q_ned_w, q_ned_x, q_ned_y, q_ned_z)
        
        # Convert yaw: NED yaw 0=North → ENU yaw 0=East
        yaw_enu = math.pi / 2.0 - yaw_ned
        
        # Reconstruct quaternion in ENU with original roll/pitch and converted yaw
        q_enu_w, q_enu_x, q_enu_y, q_enu_z = self.quaternion_from_euler(roll_ned, pitch_ned, yaw_enu)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        odom_msg.pose.pose.position.x = float(x_enu)
        odom_msg.pose.pose.position.y = float(y_enu)
        odom_msg.pose.pose.position.z = float(z_enu)
        odom_msg.pose.pose.orientation.w = float(q_enu_w)
        odom_msg.pose.pose.orientation.x = float(q_enu_x)
        odom_msg.pose.pose.orientation.y = float(q_enu_y)
        odom_msg.pose.pose.orientation.z = float(q_enu_z)
        
        odom_msg.twist.twist.linear.x = float(vx_enu)
        odom_msg.twist.twist.linear.y = float(vy_enu)
        odom_msg.twist.twist.linear.z = float(vz_enu)
        
        self.odom_pub.publish(odom_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = float(x_enu)
        t.transform.translation.y = float(y_enu)
        t.transform.translation.z = float(z_enu)
        t.transform.rotation.w = float(q_enu_w)
        t.transform.rotation.x = float(q_enu_x)
        t.transform.rotation.y = float(q_enu_y)
        t.transform.rotation.z = float(q_enu_z)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisherPx4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
