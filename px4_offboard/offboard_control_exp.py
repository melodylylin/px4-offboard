#!/usr/bin/env python

__author__ = "Zhanpeng Yang, Minhyun Cho"
__contact__ = "@purdue.edu"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommand

class OffboardControl(Node):

    def __init__(self):

        # set the name of the node
        super().__init__("px4_offboard_mocap_test")

        # set publisher and subscriber quality of service profile
        qos_profile_pub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        qos_profile_sub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # define subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile_sub)

        # define publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile_pub)

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile_pub)
        
        # self.vehicle_command_publisher = self.create_publisher(
        #     VehicleCommand, 
        #     '/fmu/in/vehicle_command', 
        #     qos_profile_pub)                                        # disable for an experiment

        # parameters for callback
        self.timer_period   =   0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)
        
        self.square = np.array([[3.0,0.0,-2.5],
                               [3.0,-6.0,-2.5],
                               [-3.0,-6.0,-2.5],
                               [-3.0,0.0,-2.5]],dtype=np.float64)
        self.pt_idx = np.uint8(0)
        self.nav_wpt_reach_rad_ =   np.float64(0.1)
        # self.counter = np.uint16(0)                                 # disable for an experiment

        self.theta  = np.float64(0.0)
        self.omega  = np.float64(1/10)

        self.cur_wpt_ = np.array([0.0,0.0,0.0],dtype=np.float64)
        self.past_wpt_ = np.array([0.0,0.0,0.0],dtype=np.float64)

        # variables for subscribers
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        self.local_pos_ned = None
        self.local_vel_ned = None

        # variables for publishers
        self.offboard_ctrl_position = False
        self.offboard_ctrl_velocity = False
        self.offboard_ctrl_acceleration = False
        self.offboard_ctrl_attitude = False
        self.offboard_ctrl_body_rate = False
        self.offboard_ctrl_actuator = False

        self.trajectory_setpoint_x = np.float64(0.0)
        self.trajectory_setpoint_y = np.float64(0.0)
        self.trajectory_setpoint_z = np.float64(0.0)
        self.trajectory_setpoint_yaw = np.float64(0.0)

    # subscriber callback
    def vehicle_status_callback(self,msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def local_position_callback(self,msg):
        self.local_pos_ned      =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)
        self.local_vel_ned      =   np.array([msg.vx,msg.vy,msg.vz],dtype=np.float64)

    # publisher
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(rclpy.clock.Clock().now().nanoseconds/1000) # time in microseconds
        msg.position = self.offboard_ctrl_position
        msg.velocity = self.offboard_ctrl_velocity
        msg.acceleration =self.offboard_ctrl_acceleration
        msg.attitude = self.offboard_ctrl_attitude
        msg.body_rate = self.offboard_ctrl_body_rate
        self.publisher_offboard_mode.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(rclpy.clock.Clock().now().nanoseconds/1000) # time in microseconds
        msg.position[0] = self.trajectory_setpoint_x
        msg.position[1] = self.trajectory_setpoint_y
        msg.position[2] = self.trajectory_setpoint_z
        msg.yaw = self.trajectory_setpoint_yaw
        self.publisher_trajectory.publish(msg)

    # def publish_vehicle_command(self,command,param1=0.0,param2=0.0):            # disable for an experiment
    #     msg = VehicleCommand()
    #     msg.param1 = param1
    #     msg.param2 = param2
    #     msg.command = command  # command ID
    #     msg.target_system = 0  # system which should execute the command
    #     msg.target_component = 1  # component which should execute the command, 0 for all components
    #     msg.source_system = 1  # system sending the command
    #     msg.source_component = 1  # component sending the command
    #     msg.from_external = True
    #     msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
    #     self.vehicle_command_publisher.publish(msg)

    def cmdloop_callback(self):

        # self.counter += 1     # disable for an experiment
        
        # if self.counter >= 10 and self.counter <= 20:     # disable for an experiment
        #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.,6.)
        #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.)
        #     self.get_logger().info("Armed and dangerous....")
        
        # publish offboard control modes
        self.offboard_ctrl_position = True
        self.publish_offboard_control_mode()

        # publish offboard position cmd
        self.trajectory_setpoint_x = self.theta*self.cur_wpt_[0]+(1-self.theta)*self.past_wpt_[0]
        self.trajectory_setpoint_y = self.theta*self.cur_wpt_[1]+(1-self.theta)*self.past_wpt_[1]
        self.trajectory_setpoint_z = self.theta*self.cur_wpt_[2]+(1-self.theta)*self.past_wpt_[2]
        self.publish_trajectory_setpoint()

        print([self.trajectory_setpoint_x,self.trajectory_setpoint_y,self.trajectory_setpoint_z])

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            if self.local_pos_ned is not None and self.local_vel_ned is not None:
                dist_xyz    =   np.sqrt(np.power(self.cur_wpt_[0]-self.local_pos_ned[0],2)+ \
                                        np.power(self.cur_wpt_[1]-self.local_pos_ned[1],2)+ \
                                        np.power(self.cur_wpt_[2]-self.local_pos_ned[2],2))

                if (self.pt_idx <= 2) and (dist_xyz <= self.nav_wpt_reach_rad_):
                    self.theta = np.float64(0.0)
                    self.past_wpt_ = self.square[self.pt_idx].flatten()
                    self.pt_idx = self.pt_idx+1
                    self.cur_wpt_ = self.square[self.pt_idx].flatten()

                elif (self.pt_idx == 3) and (dist_xyz <= self.nav_wpt_reach_rad_):
                    self.theta = np.float64(0.0)
                    self.past_wpt_ = self.square[self.pt_idx].flatten()
                    self.pt_idx = 0
                    self.cur_wpt_ = self.square[self.pt_idx].flatten()

            self.theta = self.theta+self.omega*self.timer_period
            self.theta = np.clip(self.theta,a_min=0.0,a_max=1.0)

        else:
            self.theta  = np.float64(0.0)
            self.cur_wpt_ = self.square[self.pt_idx]
            self.past_wpt_ = self.local_pos_ned


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()