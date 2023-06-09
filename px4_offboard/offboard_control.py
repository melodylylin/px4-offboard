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

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleCommand


class OffboardControl(Node):

    def __init__(self):

        super().__init__("px4_offboard")

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

        # offboard script for checking the mocap
        self.local_pos_sub = self.create_subscription(
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

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        
        self.counter = 0

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        # self.theta = 0.0
        # self.radius = 10.0
        # self.omega = 0.5

        self.square = np.array([[3.0,0.0,-2.0],
                               [3.0,-6.0,-2.0],
                               [-3.0,-6.0,-2.0],
                               [-3.0,0.0,-2.0]],dtype=np.float32)

        self.pt_idx = np.uint8(0)
        self.nav_wpt_reach_rad_ =   np.float32(0.1)

        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f"/fmu/in/vehicle_command", qos_profile) 

        self.local_pos_ned_ = None
        self.local_vel_ned_ = None

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    # offboard script for checking the mocap
    def local_position_callback(self, msg):
        self.local_pos_ned_     =   np.array([msg.x,msg.y,msg.z],dtype=np.float32)
        self.local_vel_ned_     =   np.array([msg.vx,msg.vy,msg.vz],dtype=np.float32)
    # ------------------------------------------

    def cmdloop_callback(self):
        self.counter += 1
        if self.counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Armed and dangerous....")
        
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # publish offboard position cmd
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            trajectory_msg = TrajectorySetpoint()

            trajectory_msg.position[0] = self.square[self.pt_idx][0]
            trajectory_msg.position[1] = self.square[self.pt_idx][1]
            trajectory_msg.position[2] = self.square[self.pt_idx][2]
            self.publisher_trajectory.publish(trajectory_msg)
            if self.local_pos_ned_ is not None and self.local_vel_ned_ is not None:
                dist_xyz    =   np.sqrt(np.power(trajectory_msg.position[0]-self.local_pos_ned_[0],2)+ \
                                        np.power(trajectory_msg.position[1]-self.local_pos_ned_[1],2)+ \
                                        np.power(trajectory_msg.position[2]-self.local_pos_ned_[2],2))

                if (self.pt_idx <= 2) and (dist_xyz <= self.nav_wpt_reach_rad_):
                    self.pt_idx = self.pt_idx+1

                elif (self.pt_idx == 3) and (dist_xyz <= self.nav_wpt_reach_rad_):
                    self.pt_idx = 0

        # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

        #     trajectory_msg = TrajectorySetpoint()
        #     trajectory_msg.position[0] = self.radius * np.cos(self.theta)
        #     trajectory_msg.position[1] = self.radius * np.sin(self.theta)
        #     trajectory_msg.position[2] = -5.0
        #     self.publisher_trajectory.publish(trajectory_msg)

        #     self.theta = self.theta + self.omega * self.dt

    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
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

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()