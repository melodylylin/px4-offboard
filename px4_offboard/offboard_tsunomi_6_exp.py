#!/usr/bin/env python

__author__ = "Li-Yu Lin"
__contact__ = "@purdue.edu"

import argparse

import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommand

from geometry_msgs.msg import PointStamped
from std_msgs.msg import UInt8, Bool

class OffboardMission(Node):

    def __init__(self,mode):

        super().__init__("px4_offboard_mission")

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
            '/fmu/out/vehicle_local_position',
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
       #     '/px4_1/fmu/in/vehicle_command', 
       #     qos_profile_pub)                                        # disable for an experiment

        self.detector_1 = self.create_publisher(
            PointStamped,
            '/detector/atck_act_states',
            qos_profile_pub
        )

        self.detector_2 = self.create_publisher(
            PointStamped,
            '/detector/atck_est_states',
            qos_profile_pub
        )

        self.detector_3 = self.create_publisher(
            PointStamped,
            '/detector/past_setpoint',
            qos_profile_pub
        )

        self.detector_4 = self.create_publisher(
            PointStamped,
            '/detector/true_setpoint',
            qos_profile_pub
        )

        self.detector_5 = self.create_publisher(
            PointStamped,
            '/detector/atck_setpoint',
            qos_profile_pub
        )

        self.detector_6 = self.create_publisher(
            Bool,
            '/detector/atck_engage',
            qos_profile_pub
        )

        self.detector_7 = self.create_publisher(
            Bool,
            '/detector/atck_detect',
            qos_profile_pub
        )

        self.detector_7 = self.create_publisher(
            UInt8,
            '/detector/flight_phase',
            qos_profile_pub
        )

        # parameters for callback
        self.timer_period   =   0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.flight_phase_ = np.uint8(1)
        self.entry_execute_ = np.uint8(1)

       # self.counter = np.uint16(0)                                 # disable for an experiment

        self.wpt_set_ = np.array([[0.0,0.0,-1.5],
                                  [-3.8,-2.0,-1.5]])
        
        self.theta  = np.float64(0.0)
        self.omega  = np.float64(1/10)

        self.cur_wpt_ = np.array([0.0,0.0,0.0],dtype=np.float64)
        self.past_wpt_ = np.array([0.0,0.0,0.0],dtype=np.float64)
        
        self.wpt_idx_ = np.int8(0)

        self.nav_wpt_reach_rad_ =   np.float32(0.1)     # waypoint reach condition radius

        # variables for subscribers
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        self.local_pos_ned_     =   None
        self.local_vel_ned_     =   None

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
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def local_position_callback(self,msg):
        self.local_pos_ned_      =   np.array([msg.x,msg.y,msg.z],dtype=np.float64)
        self.local_vel_ned_      =   np.array([msg.vx,msg.vy,msg.vz],dtype=np.float64)

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

       # self.counter += 1   # disable for an experiment

       # if self.counter >= 10 and self.counter <= 20:
       #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
       #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0)
       #     self.get_logger().info("Armed and dangerous....")

        # publish offboard control modes
        self.offboard_ctrl_position = True
        self.publish_offboard_control_mode()

        # publish offboard position cmd
        # phase 1: engage offboard control - switch to offboard/position mode
        # hold its position at a starting point
        # proceed to offboard wpt mission when the multicoptor reaches a setpoint
        if (self.flight_phase_ == 1) and (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):

            # entry:
            if self.entry_execute_:

                self.entry_execute_  = 	0
                self.cur_wpt_  = self.wpt_set_[0]#np.array([4.0,-2.0,-1.2],dtype=np.float64)
                self.past_wpt_ = self.local_pos_ned_
                self.theta = np.float64(0.0)

            # during:
            print("Current Mode: Offboard (Position hold at a starting point)")
            self.trajectory_setpoint_x = self.theta*self.cur_wpt_[0]+(1-self.theta)*self.past_wpt_[0]
            self.trajectory_setpoint_y = self.theta*self.cur_wpt_[1]+(1-self.theta)*self.past_wpt_[1]
            self.trajectory_setpoint_z = self.theta*self.cur_wpt_[2]+(1-self.theta)*self.past_wpt_[2]
            self.trajectory_setpoint_yaw  =   np.float64(-np.pi/2)
            self.publish_trajectory_setpoint()

            self.theta = self.theta+self.omega*self.timer_period
            self.theta = np.clip(self.theta,a_min=0.0,a_max=1.0)

            # transition
            if (self.local_pos_ned_ is not None) and (self.local_vel_ned_ is not None):
                dist_xyz    =   np.sqrt(np.power(self.cur_wpt_[0]-self.local_pos_ned_[0],2)+ \
                                        np.power(self.cur_wpt_[1]-self.local_pos_ned_[1],2)+ \
                                        np.power(self.cur_wpt_[2]-self.local_pos_ned_[2],2))

                if dist_xyz < self.nav_wpt_reach_rad_:

                    # exit:
                    self.flight_phase_     =	2
                    self.entry_execute_    =	1

        # phase 2: engage offboard wpt mission - hold offboard/position mode
        # perform a waypoint mission and use a detector for spoofing attack
        # exit when the offboard mode control turns into off
        elif (self.flight_phase_ == 2) and (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):

            # entry:
            if self.entry_execute_:

                self.entry_execute_ 	    = 	0
                self.wpt_idx_               =   np.uint8(0)
                self.past_wpt_              =   self.local_pos_ned_ #np.array([0.0,0.0,-1.2],dtype=np.float64)
                self.cur_wpt_               =   self.wpt_set_[self.wpt_idx_].flatten() #np.add(self.past_wpt_,np.array([0.0,-6.0,0.0],dtype=np.float64))
                self.theta  = np.float64(0.0)

            # during:
            if (self.local_pos_ned_ is not None) and (self.local_vel_ned_ is not None):

                print("Current Mode: Offboard (wpt mission)")

                self.trajectory_setpoint_x = self.theta*self.cur_wpt_[0]+(1-self.theta)*self.past_wpt_[0]
                self.trajectory_setpoint_y = self.theta*self.cur_wpt_[1]+(1-self.theta)*self.past_wpt_[1]
                self.trajectory_setpoint_z = self.theta*self.cur_wpt_[2]+(1-self.theta)*self.past_wpt_[2]
                self.trajectory_setpoint_yaw  =   np.float64(-np.pi/2)
                self.publish_trajectory_setpoint()

                self.theta = self.theta+self.omega*self.timer_period
                self.theta = np.clip(self.theta,a_min=0.0,a_max=1.0)

                # transition
                dist_xyz    =   np.sqrt(np.power(self.cur_wpt_[0]-self.local_pos_ned_[0],2)+ \
                                        np.power(self.cur_wpt_[1]-self.local_pos_ned_[1],2)+ \
                                        np.power(self.cur_wpt_[2]-self.local_pos_ned_[2],2))
                
                if (self.wpt_idx_ <= self.wpt_set_.shape[0] - 1) and (dist_xyz <= self.nav_wpt_reach_rad_):
                    self.past_wpt_ = self.wpt_set_[self.wpt_idx_].flatten()
                    self.wpt_idx_ = self.wpt_idx_+1
                    self.cur_wpt_ = self.wpt_set_[self.wpt_idx_]

                elif (self.wpt_idx_ == self.wpt_set_.shape[0] ) and (dist_xyz <= self.nav_wpt_reach_rad_):
                    # exit:
                    print("Offboard mission finished")
            
        else:
            self.flight_phase_     =	1
            self.entry_execute_    =	1

            self.theta  = np.float64(0.0)
            if (self.local_pos_ned_ is not None):
                self.past_wpt_ = self.local_pos_ned_

            else:
                self.past = np.array([0.0,0.0,0.0],dtype=np.float64)

            self.cur_wpt_ = self.wpt_set_[0] #np.array([0.0,0.0,-1.2],dtype=np.float64)

            self.trajectory_setpoint_x = self.theta*self.cur_wpt_[0]+(1-self.theta)*self.past_wpt_[0]
            self.trajectory_setpoint_y = self.theta*self.cur_wpt_[1]+(1-self.theta)*self.past_wpt_[1]
            self.trajectory_setpoint_z = self.theta*self.cur_wpt_[2]+(1-self.theta)*self.past_wpt_[2]
            self.trajectory_setpoint_yaw  =   np.float64(-np.pi/2)
            self.publish_trajectory_setpoint()

        stamp = self.get_clock().now().to_msg()

        flight_phase_pub = UInt8()
        flight_phase_pub.data = self.flight_phase_
        self.detector_7.publish(flight_phase_pub)

def main():
    parser = argparse.ArgumentParser(description='Delivering parameters for tests')
    parser.add_argument('--mode','-m',type=int,default=np.uint8(1),help='mode setting')
    argin = parser.parse_args()

    rclpy.init(args=None)

    offboard_mission = OffboardMission(argin.mode)

    rclpy.spin(offboard_mission)

    offboard_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
