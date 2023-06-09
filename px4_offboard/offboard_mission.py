#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import UInt8, Bool

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition


class OffboardMission(Node):

    def __init__(self):

        super().__init__("px4_offboard_mission")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        # offboard script for verifying the detector
        self.local_pos_sub = self.create_sbuscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile)
        # ------------------------------------------

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile)

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile)

        self.detector_1 = self.create_publisher(
            Vector3Stamped,
            '/detector/atck_act_states',
            qos_profile
        )

        self.detector_2 = self.create_publisher(
            Vector3Stamped,
            '/detector/atck_est_states',
            qos_profile
        )

        self.detector_3 = self.create_publisher(
            Vector3Stamped,
            '/detector/past_setpoint',
            qos_profile
        )

        self.detector_4 = self.create_publisher(
            Vector3Stamped,
            '/detector/true_setpoint',
            qos_profile
        )

        self.detector_5 = self.create_publisher(
            Vector3Stamped,
            '/detector/atck_setpoint',
            qos_profile
        )

        self.detector_6 = self.create_publisher(
            Bool,
            '/detector/atck_engage',
            qos_profile
        )

        self.detector_7 = self.create_publisher(
            Bool,
            '/detector/atck_detect',
            qos_profile
        )

        self.detector_7 = self.create_publisher(
            UInt8,
            '/detector/flight_phase',
            qos_profile
        )

        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # offboard script for verifying the detector
        self.local_pos_ned_     =   None #np.array([0.0,0.0,0.0],dtype=np.float32)
        self.local_vel_ned_     =   None #np.array([0.0,0.0,0.0],dtype=np.float32)

        self.flight_phase_      =   np.uint8(1)
        self.entry_execute_     =   np.uint8(1)

        self.past_setpoint_     =   np.array([0.0,0.0,0.0],dtype=np.float32)
        self.true_setpoint_     =   np.array([0.0,0.0,0.0],dtype=np.float32)
        self.atck_setpoint_     =   np.array([0.0,0.0,0.0],dtype=np.float32)

        self.atck_engage_       =   False
        self.atck_detect_       =   True

        self.atck_act_states_   =   np.array([0.0,0.0,0.0],dtype=np.float32)
        self.atck_est_states_   =   np.array([0.0,0.0,0.0],dtype=np.float32)

        # waypoint reach condition radius
        self.nav_wpt_reach_rad_ =   np.float32(0.1)

        # observer gain
        self.Lobv               =   np.array([[0.62,0.00,0.00],
                                              [0.00,0.62,0.00],
                                              [0.00,0.00,0.62]])

        # detector threshold
        self.detect_threshold   =   np.float(1.00)                                      
        # ------------------------------------------

        # disable when doing an experiment
        self.counter = 0

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    # offboard script for verifying the detector
    def local_position_callback(self, msg):
        self.local_pos_ned_     =   np.array([msg.x,msg.y,msg.z],dtype=np.float32)
        self.local_vel_ned_     =   np.array([msg.vx,msg.vy,msg.vz],dtype=np.float32)
    # ------------------------------------------

    # disable when doing an experiment
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 0  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def cmdloop_callback(self):

        # disable when doing an experiment
        if self.counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0)
            self.get_logger().info("Armed and dangerous....")

        # publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds/1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # publish offboard position cmd
        # phase 1: engage offboard control - switch to offboard/position mode
        # hold its position at a starting point
        # proceed to offboard wpt mission when the multicoptor reaches a setpoint
        if (self.flight_phase_ == 1) and (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):

            # entry:
            if self.entry_execute_:

                self.entry_execute_ 	    = 	0
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.position[0]  =   np.float32(0.0)
                trajectory_msg.position[1]  =   np.float32(0.0)
                trajectory_msg.position[2]  =   np.float32(-2.0)
                trajectory_msg.yaw          =   0

            # during:
            print("Current Mode: Offboard (Position hold at a starting point)")
            self.publisher_trajectory.publish(trajectory_msg)

            # transition
            if (self.local_pos_ned_ is not None) and (self.local_vel_ned_ is not None):
                dist_xyz    =   np.sqrt(np.power(trajectory_msg.position[0]-self.local_pos_ned_[0],2)+ \
                                        np.power(trajectory_msg.position[1]-self.local_pos_ned_[1],2)+ \
                                        np.power(trajectory_msg.position[2]-self.local_pos_ned_[2],2))

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

                self.past_setpoint_         =   [trajectory_msg.position[0],trajectory_msg.position[1],trajectory_msg.position[2]]
                self.true_setpoint_         =   np.add(self.past_setpoint_,np.array([0.0,-6.0,0.0],dtype=np.float32))
                self.atck_setpoint_         =   np.add(self.past_setpoint_,np.array([3.0,-6.0,0.0],dtype=np.float32))
                self.atck_engage_           =   True
                self.atck_detect_           =   False

            # during:
            print("Current Mode: Offboard (wpt mission to test the detector)")
            alpha       =   (self.local_pos_ned_[1]-self.past_setpoint_[1])/ \
                            (self.true_setpoint_[1]-self.past_setpoint_[1])
            alpha       =   np.clip(2*alpha,a_min = 0,a_max = 1)

            if self.atck_engage_ and not self.atck_detect_:
                alpha   =   alpha
            else:
                alpha   =   0

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0]  =   (1-alpha)*self.true_setpoint_[0]+alpha*self.atck_setpoint_[0]
            trajectory_msg.position[1]  =   (1-alpha)*self.true_setpoint_[1]+alpha*self.atck_setpoint_[1]
            trajectory_msg.position[2]  =   (1-alpha)*self.true_setpoint_[2]+alpha*self.atck_setpoint_[2]
            trajectory_msg.yaw          =   0
            self.publisher_trajectory.publish(trajectory_msg)

            proj_atck   =   get_projection_matrix(self.true_setpoint_-self.atck_setpoint_)

            self.atck_act_states_       =   self.local_pos_ned_
            ack_vec                     =   (np.matmul(np.atleast_2d(self.atck_act_states_),proj_atck)).flatten()

            self.atck_est_states_       =   (np.matmul(np.atleast_2d(self.atck_est_states_),(np.eye(3,dtype=np.float32)-self.Lobv).T)).flatten()+ \
                                            (np.matmul(np.atleast_2d(self.local_vel_ned_),(np.eye(3,dtype=np.float32)-proj_atck).T)).flatten()*self.timer_period+ \
                                            (np.matmul(np.atleast_2d(self.atck_act_states_),(self.Lobv).T)).flatten()- \
                                            (np.matmul(np.atleast_2d(ack_vec),(self.Lobv).T)).flatten()

            if np.linalg.norm(self.atck_est_states_-self.atck_act_states_) >= self.detect_threshold:
                self.atck_engage_       =   False
                self.atck_detect_       =   True

            # transition
            dist_xyz    =   np.sqrt(np.power(self.true_setpoint_[0]-self.local_pos_ned_[0],2)+ \
                                       np.power(self.true_setpoint_[1]-self.local_pos_ned_[1],2)+ \
                                       np.power(self.true_setpoint_[2]-self.local_pos_ned_[2],2))

            if dist_xyz < self.nav_wpt_reach_rad_:

                # exit:
                print("Offboard mission finished")

        else:
            self.flight_phase_     =	1
            self.entry_execute_    =	1

        atck_act_states_pub = Vector3Stamped()
        atck_act_states_pub.x = self.atck_act_states_[0]
        atck_act_states_pub.y = self.atck_act_states_[1]
        atck_act_states_pub.z = self.atck_act_states_[2]
        atck_act_states_pub.timestamp = int(Clock().now().nanoseconds/1000)
        self.detector_1.publish(atck_act_states_pub)

        atck_est_states_pub = Vector3Stamped()
        atck_est_states_pub.x = self.atck_est_states_[0]
        atck_est_states_pub.y = self.atck_est_states_[1]
        atck_est_states_pub.z = self.atck_est_states_[2]
        atck_est_states_pub.timestamp = int(Clock().now().nanoseconds/1000)
        self.detector_2.publish(atck_est_states_pub)

        atck_past_setpoint_pub = Vector3Stamped()
        atck_past_setpoint_pub.x = self.past_setpoint_[0]
        atck_past_setpoint_pub.y = self.past_setpoint_[1]
        atck_past_setpoint_pub.z = self.past_setpoint_[2]
        atck_past_setpoint_pub.timestamp = int(Clock().now().nanoseconds/1000)
        self.detector_3.publish(atck_past_setpoint_pub)

        atck_true_setpoint_pub = Vector3Stamped()
        atck_true_setpoint_pub.x = self.true_setpoint_[0]
        atck_true_setpoint_pub.y = self.true_setpoint_[1]
        atck_true_setpoint_pub.z = self.true_setpoint_[2]
        atck_true_setpoint_pub.timestamp = int(Clock().now().nanoseconds/1000)
        self.detector_4.publish(atck_true_setpoint_pub)

        atck_atck_setpoint_pub = Vector3Stamped()
        atck_atck_setpoint_pub.x = self.atck_setpoint_[0]
        atck_atck_setpoint_pub.y = self.atck_setpoint_[1]
        atck_atck_setpoint_pub.z = self.atck_setpoint_[2]
        atck_atck_setpoint_pub.timestamp = int(Clock().now().nanoseconds/1000)
        self.detector_5.publish(atck_atck_setpoint_pub)

        atck_engage_pub = Bool()
        atck_engage_pub.data = self.atck_engage_
        self.detector_6.publish(atck_engage_pub)

        atck_detect_pub = Bool()
        atck_detect_pub.data = self.atck_detect_
        self.detector_6.publish(atck_detect_pub)

        flight_phase_pub = UInt8()
        flight_phase_pub.data = self.flight_phase_
        self.detector_7.publish(flight_phase_pub)

def get_projection_matrix(vector):

    proj_mat    =   np.matmul(np.atleast_2d(vector).T,np.atleast_2d(vector))
    proj_mat    =   proj_mat/np.dot(vector,vector)

    return proj_mat

def main(args=None):
    rclpy.init(args=args)

    offboard_mission = OffboardMission()

    rclpy.spin(offboard_mission)

    offboard_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
