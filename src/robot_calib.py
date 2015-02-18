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

import sys
import roslib
roslib.load_manifest('rosban')
import rospy


from math import radians, degrees

# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState

from dynamixel_msgs.msg import *  # import dynamixel_msgs node's messages
from dynamixel_controllers.srv import *

from std_msgs.msg import Float64

from xml.dom import minidom
# from arbotix_msgs.srv import Relax
# from arbotix_python.joints import *

INIT_TORQUE = 0.0
OFF_TORQUE = 0.0
ON_TORQUE = 0.4
TIMER_DT = 10

width = 325


class Robot():

    def __init__(self):

        i = 0
        dynamixels_names = rospy.get_param('/dyn_controllers/', dict())
        dynamixels_params = rospy.get_param('/dynamixel/', dict())

        self.params = dict()

        for c in dynamixels_params.keys():
            for i in dynamixels_params[c]['connected_ids']:
                # print dynamixels_params[c][str(i)]
                # print i
                # print dynamixels_params[c].keys()
                self.params[i] = dynamixels_params[c][str(i)]

        dynamixels = dict()

        for name in dynamixels_names['names']:
            # print name
            dynamixels[name] = rospy.get_param(name, dict())

        self.servos = dict()
        self.servos_pos = dict()
        self.publishers = dict()
        self.subscribers = dict()
        self.services = dict()

        self.motors = rospy.Subscriber(
            '/motor_states/django_port', MotorStateList, self.motorCb)

        i = 0
        for name in sorted(dynamixels.keys()):
        # for name in sorted(joints.keys()):
            # pull angles
            # min_angle, max_angle = getJointLimits(name, joint_defaults)
            init_angle_raw = dynamixels[name]['motor']['init']
            # print params
            min_angle = (dynamixels[name]['motor']['min'] - init_angle_raw) / self.params[
                int(dynamixels[name]['motor']['id'])]['encoder_ticks_per_radian']
            max_angle = (dynamixels[name]['motor']['max'] - init_angle_raw) / self.params[
                int(dynamixels[name]['motor']['id'])]['encoder_ticks_per_radian']

            # in degrees?
            # min_angle = degrees(min_angle)
            # max_angle = degrees(max_angle)

            # min_angle = dynamixels[name]['motor']['min']/params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']
            # max_angle =
            # dynamixels[name]['motor']['max']/params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']
            init_angle = dynamixels[name]['motor']['init'] / self.params[
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

    def motorCb(self, msg):
        # print str(msg)
        # for i in len(msg):
        # for m in msg.motor_states:
        #     rospy.loginfo(
        # str(self.servos[m.id]) + ' ' + str(m.id) + ' ' + str(m.position) + '
        # ' + str(self.servos_pos[m.id]) + ' ' +
        # str(degrees(self.servos_pos[m.id])))
        pass

    def zero(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            for s in self.servos.keys():
                # d = self.servos[s].getPosition()
                self.services[s](ON_TORQUE)
                self.publishers[s].publish(0.0)
            rate.sleep()

    def dump(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            for s in self.servos.keys():
                # d = self.servos[s].getPosition()
                # self.services[s](ON_TORQUE)
                # self.publishers[s].publish(0.0)
                l = s + ' ' + str(self.servos_pos[s])
                rospy.loginfo(l)
            rate.sleep()

    def to_raw(self, val, id):
        # print self.params
        print id, val, self.params[id]['encoder_ticks_per_radian']
        return val * self.params[id]['encoder_ticks_per_radian']

if __name__ == '__main__':
    # initialize GUI

    offsets = {}
    xmldoc = minidom.parse(sys.argv[1])
    servolist = xmldoc.getElementsByTagName('ServoConfig')
    # print servolist
    for s in servolist:
        # print s.toxml()
        servoid = s.getElementsByTagName('Id')
        servoangle = s.getElementsByTagName('ZeroAngle')
        servotype = s.getElementsByTagName('Type')

        print servoid[0].firstChild.nodeValue, servoangle[0].firstChild.nodeValue
        if servotype[0].firstChild.nodeValue != 'None':
            offsets[servoid[0].firstChild.nodeValue] = servoangle[
                0].firstChild.nodeValue

    print offsets

    rospy.init_node('Init_robot')
    r = Robot()
    # r.zero()

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     rate.sleep()
    offsets_raw = {}
    for k in offsets.keys():
        offsets_raw[k] = r.to_raw(radians(float(offsets[k])), int(k))

    print offsets_raw
