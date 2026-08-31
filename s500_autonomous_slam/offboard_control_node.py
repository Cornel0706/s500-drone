#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
import math
from enum import Enum

class State(Enum):
    IDLE = 1
    PRE_ARM = 2
    ARMING = 3
    TAKING_OFF = 4
    HOVERING = 5
    OFFBOARD_ACTIVE = 6
    LANDING = 7

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # Parameters
        self.declare_parameter('takeoff_altitude', 2.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('auto_takeoff', True)

        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.auto_takeoff = self.get_parameter('auto_takeoff').value

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odom_callback, qos_profile)

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # State
        self.state = State.IDLE
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.current_yaw = 0.0
        self.current_pos = [0.0, 0.0, 0.0]
        self.takeoff_start_pos = [0.0, 0.0, 0.0]
        
        self.cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()
        self.state_timer_start = self.get_clock().now()

        # Timer
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Offboard Control Node Initialized.')
        
        if self.auto_takeoff:
            self.transition_to(State.PRE_ARM)

    def transition_to(self, new_state):
        self.get_logger().info(f'State transition: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_timer_start = self.get_clock().now()

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_odom_callback(self, msg):
        # Extract yaw from quaternion (NED)
        q = msg.q
        self.current_yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
        self.current_pos = [msg.position[0], msg.position[1], msg.position[2]]

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()
        if self.state == State.HOVERING:
            self.transition_to(State.OFFBOARD_ACTIVE)

    def timer_callback(self):
        # Always publish offboard control mode
        self.publish_offboard_control_mode()

        now = self.get_clock().now()
        elapsed = (now - self.state_timer_start).nanoseconds / 1e9
        
        # Check cmd_vel timeout
        if self.state == State.OFFBOARD_ACTIVE:
            cmd_elapsed = (now - self.last_cmd_vel_time).nanoseconds / 1e9
            if cmd_elapsed > 1.0:
                self.get_logger().warn('cmd_vel timeout. Hovering.')
                self.transition_to(State.HOVERING)

        if self.state == State.IDLE:
            pass
            
        elif self.state == State.PRE_ARM:
            self.publish_hover_setpoint()
            if elapsed > 2.0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # SET_MODE to offboard
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # ARM
                self.takeoff_start_pos = self.current_pos
                self.transition_to(State.ARMING)
                
        elif self.state == State.ARMING:
            self.publish_hover_setpoint()
            if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.transition_to(State.TAKING_OFF)
                
        elif self.state == State.TAKING_OFF:
            target_z = self.takeoff_start_pos[2] - self.takeoff_altitude  # NED z is down
            self.publish_position_setpoint(self.takeoff_start_pos[0], self.takeoff_start_pos[1], target_z)
            if abs(self.current_pos[2] - target_z) < 0.2:
                self.transition_to(State.HOVERING)
                
        elif self.state == State.HOVERING:
            self.publish_hover_setpoint()
            
        elif self.state == State.OFFBOARD_ACTIVE:
            self.publish_velocity_setpoint_from_cmd_vel()
            
        elif self.state == State.LANDING:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.transition_to(State.IDLE)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = (self.state == State.TAKING_OFF)
        msg.velocity = (self.state in [State.PRE_ARM, State.ARMING, State.HOVERING, State.OFFBOARD_ACTIVE])
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_hover_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = self.current_yaw
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_velocity_setpoint_from_cmd_vel(self):
        # cmd_vel in FLU
        vx_flu = max(min(self.cmd_vel.linear.x, self.max_velocity), -self.max_velocity)
        vy_flu = max(min(self.cmd_vel.linear.y, self.max_velocity), -self.max_velocity)
        vz_flu = max(min(self.cmd_vel.linear.z, self.max_velocity), -self.max_velocity)
        
        # FLU to FRD
        vx_frd = vx_flu
        vy_frd = -vy_flu
        vz_frd = -vz_flu
        
        # FRD to NED
        yaw = self.current_yaw
        vx_ned = vx_frd * math.cos(yaw) - vy_frd * math.sin(yaw)
        vy_ned = vx_frd * math.sin(yaw) + vy_frd * math.cos(yaw)
        vz_ned = vz_frd
        
        yawspeed_ned = -self.cmd_vel.angular.z
        
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [vx_ned, vy_ned, vz_ned]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = yawspeed_ned
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
