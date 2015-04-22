#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: 'robot_gui.py'
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen.000@gmail.com
#  Created	:
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
#


import roslib
roslib.load_manifest('rosban')
import rospy


from math import radians, degrees
import numpy as np
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState

from dynamixel_msgs.msg import *  # import dynamixel_msgs node's messages
from dynamixel_controllers.srv import *

from std_msgs.msg import Float64

# from arbotix_msgs.srv import Relax
# from arbotix_python.joints import *

INIT_TORQUE = 0.0
OFF_TORQUE = 0.0
ON_TORQUE = 1.0
TIMER_DT = 10

width = 325


class Robot():

    def __init__(self):

        i = 0
        dynamixels_names = rospy.get_param('/dyn_controllers/', dict())
        dynamixels_params = rospy.get_param('/dynamixel/', dict())

        params = dict()

        for c in dynamixels_params.keys():
            for i in dynamixels_params[c]['connected_ids']:
                # print dynamixels_params[c][str(i)]
                # print i
                # print dynamixels_params[c].keys()
                params[i] = dynamixels_params[c][str(i)]

        dynamixels = dict()
        self.dynamixels = dict()
        for name in dynamixels_names['names']:
            # print name
            dynamixels[name] = rospy.get_param(name, dict())
            self.dynamixels[name] = rospy.get_param(name, dict())

        self.servos = dict()
        self.servos_pos = dict()
        self.publishers = dict()
        self.subscribers = dict()
        self.services = dict()

        i = 0
        for name in sorted(dynamixels.keys()):
        # for name in sorted(joints.keys()):
            # pull angles
            # min_angle, max_angle = getJointLimits(name, joint_defaults)
            init_angle_raw = dynamixels[name]['motor']['init']
            # print params
            min_angle = (dynamixels[name]['motor']['min'] - init_angle_raw) / params[
                int(dynamixels[name]['motor']['id'])]['encoder_ticks_per_radian']
            max_angle = (dynamixels[name]['motor']['max'] - init_angle_raw) / params[
                int(dynamixels[name]['motor']['id'])]['encoder_ticks_per_radian']

            # in degrees?
            # min_angle = degrees(min_angle)
            # max_angle = degrees(max_angle)

            # min_angle = dynamixels[name]['motor']['min']/params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']
            # max_angle =
            # dynamixels[name]['motor']['max']/params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']
            init_angle = dynamixels[name]['motor']['init'] / params[
                dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']

            # init_angle = degrees(init_angle)

            # init_angle=0.0

            # print
            # min_angle,max_angle,init_angle,dynamixels[name]['motor']['min'],dynamixels[name]['motor']['max'],params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']

            # create publisher
            self.publishers[dynamixels[name]['motor']['id']] = rospy.Publisher(
                name + '/command', Float64)
            # if rospy.get_param('/arbotix/joints/'+name+'/type','dynamixel') == 'dynamixel':
            #     self.relaxers.append(rospy.ServiceProxy(name+'/relax', Relax))
            # else:
            #     self.relaxers.append(None)

            # create subscriber
            self.subscribers[dynamixels[name]['motor']['id']] = rospy.Subscriber(
                name + '/state', JointState, self.stateCb)

            # create service proxy
            self.services[dynamixels[name]['motor']['id']] = rospy.ServiceProxy(
                name + '/set_torque_limit', SetTorqueLimit)

            self.services[dynamixels[name]['motor']['id']](INIT_TORQUE)

            self.servos[dynamixels[name]['motor']['id']] = dynamixels[
                name]['joint_name']
            self.servos_pos[dynamixels[name]['motor']['id']] = init_angle
            i += 1

    def stateCb(self, msg):
        self.servos_pos[msg.motor_ids[0]] = msg.current_pos

    def zero(self):
        rate = rospy.Rate(10)
        torque = 0.1
        stop = False
        nb = 0.0
        while not rospy.is_shutdown() and not stop:

            for s in self.servos.keys():
                # d = self.servos[s].getPosition()
                self.services[s](torque)
                self.publishers[s].publish(0.0)
            torque = min(ON_TORQUE, torque + 0.1)
            if torque == 1.0:
                print "TORQUE MAX"
                stop = True

            else:
                print "TORQUE: ", torque

            if nb > 10.0:
                stop = True
            nb += 0.1

            rate.sleep()

    def down(self, angle, startangle=0.0, duration=5.0, freq=30.0):
        knee_angle = startangle

        t = np.linspace(0.0, duration, freq * duration)

        motors = ['lleg_h_p_controller',
                  'lleg_k_p_controller', 'lleg_a_p_controller', 'rleg_h_p_controller',
                  'rleg_k_p_controller', 'rleg_a_p_controller']

        for tt in t:
            # for name in motors:
            #     self.publishers[
            #         self.dynamixels[name]['motor']['id']].publish(knee_angle)

            self.publishers[
                self.dynamixels['rleg_h_p_controller']['motor']['id']].publish(knee_angle)
            self.publishers[
                self.dynamixels['rleg_k_p_controller']['motor']['id']].publish(- 2.0 * knee_angle)

            self.publishers[
                self.dynamixels['rleg_a_p_controller']['motor']['id']].publish(- knee_angle)

            self.publishers[
                self.dynamixels['lleg_h_p_controller']['motor']['id']].publish(- knee_angle)
            self.publishers[
                self.dynamixels['lleg_k_p_controller']['motor']['id']].publish(2.0 * knee_angle)

            self.publishers[
                self.dynamixels['lleg_a_p_controller']['motor']['id']].publish(knee_angle)

            knee_angle += (angle - startangle) / len(t)
            rospy.sleep(1.0 / freq)

    def oneleg(self):
        torque = 0.2
        self.services[
            self.dynamixels['larm_s_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['lleg_h_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['rarm_s_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['rleg_a_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['rleg_h_r_controller']['motor']['id']](torque)

        rospy.sleep(1.0)

        self.publishers[
            self.dynamixels['larm_s_r_controller']['motor']['id']].publish(- 0.16)
        self.publishers[
            self.dynamixels['lleg_h_r_controller']['motor']['id']].publish(0.19)
        self.publishers[
            self.dynamixels['rarm_s_r_controller']['motor']['id']].publish(1.55)
        self.publishers[
            self.dynamixels['rleg_a_r_controller']['motor']['id']].publish(- 0.04)
        self.publishers[
            self.dynamixels['rleg_h_r_controller']['motor']['id']].publish(0.226)
        rospy.sleep(3.0)
        torque = 1.0

        self.services[
            self.dynamixels['larm_s_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['lleg_h_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['rarm_s_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['rleg_a_r_controller']['motor']['id']](torque)
        self.services[
            self.dynamixels['rleg_h_r_controller']['motor']['id']](torque)

        rospy.sleep(1.0)

    def lleg_shoot(self):

        rate = rospy.Rate(50)
        angle = 0.0
        freq = 1.0
        # amp = 0.3
        amp = 0.0
        while not rospy.is_shutdown():
            if amp < 0.3:
                amp += 0.0001
            angle = amp * np.sin(2.0 * np.pi * freq * rospy.get_time())
            self.publishers[
                self.dynamixels['lleg_h_p_controller']['motor']['id']].publish(1.5 * angle)
            self.publishers[
                self.dynamixels['lleg_a_p_controller']['motor']['id']].publish(1.5 * angle)

            self.publishers[
                self.dynamixels['rleg_a_p_controller']['motor']['id']].publish(- 0.1 * angle + 0.026)
            rate.sleep()

    def up(self, angle, startangle=0.0, duration=5.0, freq=30.0):
        knee_angle = startangle

        t = np.linspace(0.0, duration, freq * duration)

        motors = ['lleg_h_p_controller',
                  'lleg_k_p_controller', 'lleg_a_p_controller', 'rleg_h_p_controller',
                  'rleg_k_p_controller', 'rleg_a_p_controller']

        for tt in t:
            # for name in motors:
            #     self.publishers[
            #         self.dynamixels[name]['motor']['id']].publish(knee_angle)

            self.publishers[
                self.dynamixels['rleg_h_p_controller']['motor']['id']].publish(knee_angle)
            self.publishers[
                self.dynamixels['rleg_k_p_controller']['motor']['id']].publish(- 2.0 * knee_angle)

            self.publishers[
                self.dynamixels['rleg_a_p_controller']['motor']['id']].publish(- knee_angle)

            self.publishers[
                self.dynamixels['lleg_h_p_controller']['motor']['id']].publish(- knee_angle)
            self.publishers[
                self.dynamixels['lleg_k_p_controller']['motor']['id']].publish(2.0 * knee_angle)

            self.publishers[
                self.dynamixels['lleg_a_p_controller']['motor']['id']].publish(knee_angle)

            knee_angle -= (startangle - angle) / len(t)
            rospy.sleep(1.0 / freq)


if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('Init_robot')
    r = Robot()
    r.zero()
    # ANGLE = 60.0
    # DURATION = 1.0
    # r.down(radians(ANGLE), duration=DURATION)
    # r.up(0.0, startangle=radians(ANGLE), duration=DURATION)
    r.oneleg()
    r.lleg_shoot()
