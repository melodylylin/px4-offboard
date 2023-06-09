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

        # self.mocap_odom_pub_ = self.create_publisher(
        #     VehicleOdometry,
        #     '/fmu/in/vehicle_visual_odometry',
        #     qos_profile
        # )

        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # offboard script for verifying the detector
        self.local_pos_ned_     =   np.array([0.0,0.0,0.0],dtype=np.float32)
        self.local_vel_ned_     =   np.array([0.0,0.0,0.0],dtype=np.float32)

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
        self.detect_threshold   =   numpy.float(1.00)                                      
        # ------------------------------------------

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

    def cmdloop_callback(self):

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
                trajectory_msg.position[0]  =   numpy.float32(0.0)
                trajectory_msg.position[1]  =   numpy.float32(0.0)
                trajectory_msg.position[2]  =   numpy.float32(-2.0)
                trajectory_msg.yaw          =   0

            # during:
            print("Current Mode: Offboard (Position hold at a starting point)")
            self.publisher_trajectory.publish(trajectory_msg)

            # transition
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
                self.true_setpoint_         =   np.add(self.past_setpoint_,np.array([0.0,-6.0,0.0],dtype=numpy.float32))
                self.atck_setpoint_         =   np.add(self.past_setpoint_,np.array([3.0,-6.0,0.0],dtype=numpy.float32))
                self.atck_engage_           =   True
                self.atck_detect_           =   False

            # during:
            print("Current Mode: Offboard (wpt mission to test the detector)")
            alpha       =   (self.local_pos_ned_[1]-self.past_setpoint_[1])/ \
                            (self.true_setpoint_[1]-self.past_setpoint_[1])
            alpha       =   numpy.clip(2*alpha,a_min = 0,a_max = 1)

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

            proj_atck   =   get_projection_matrix(self.true_setpoint_[idx]-self.atck_setpoint_[idx])

            self.atck_act_states_       =   self.local_pos_ned_
            ack_vec                     =   (np.matmul(np.atleast_2d(self.atck_act_states_),proj_atck)).flatten()

            self.atck_est_states_       =   (np.matmul(np.atleast_2d(self.atck_est_states_),(np.eye(3,dtype=numpy.float32)-self.Lobv).T)).flatten()+ \
                                            (np.matmul(np.atleast_2d(self.local_vel_ned_),(np.eye(3,dtype=numpy.float32)-proj_atck).T)).flatten()*self.timer_period+ \
                                            (np.matmul(np.atleast_2d(self.atck_act_states_),(self.Lobv).T)).flatten()- \
                                            (np.matmul(np.atleast_2d(ack_vec),(self.Lobv).T)).flatten()

            if np.linalg.norm(self.atck_est_states_-self.atck_act_states_) >= self.detect_threshold:
                self.atck_engage_       =   False
                self.atck_detect_       =   True

            # transition
            dist_xyz    =   numpy.sqrt(numpy.power(true_setpoint_[0]-self.local_pos_ned_[0],2)+ \
                                       numpy.power(true_setpoint_[1]-self.local_pos_ned_[1],2)+ \
                                       numpy.power(true_setpoint_[2]-self.local_pos_ned_[2],2))


            if dist_xyz < self.nav_wpt_reach_rad_:

                # exit:
                print("Offboard mission finished")

        else:
            self.flight_phase_     =	1
            self.entry_execute_    =	1

def get_projection_matrix(vector):

    proj_mat    =   numpy.matmul(numpy.atleast_2d(vector).T,numpy.atleast_2d(vector))
    proj_mat    =   proj_mat/numpy.dot(vector,vector)

    return proj_mat

def main(args=None):
    rclpy.init(args=args)

    offboard_mission = OffboardMission()

    rclpy.spin(offboard_mission)

    offboard_mission.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
