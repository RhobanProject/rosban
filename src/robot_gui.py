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
import wx
import time
from math import radians, degrees
import sys
import argparse

# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState

from dynamixel_msgs.msg import *  # import dynamixel_msgs node's messages
from dynamixel_controllers.srv import *

from std_msgs.msg import Float64

# from arbotix_msgs.srv import Relax
# from arbotix_python.joints import *

INIT_TORQUE = 0.0
OFF_TORQUE = 0.0
ON_TORQUE = 0.5
TIMER_DT = 10

width = 325


class servoSlider():

    def __init__(self, parent, min_angle, max_angle, name, i, default_enabled=False):
        self.name = name
        if name.find("_controller") > 0:  # remove _controller for display name
            name = name[0:-11]
        self.position = wx.Slider(parent, -1, 0, int(min_angle * 100), int(
            max_angle * 100), wx.DefaultPosition, (150, -1), wx.SL_HORIZONTAL | wx.SL_AUTOTICKS)

        # self.position.SetTickFreq(5, 1)
        self.enabled = wx.CheckBox(parent, i, name + ":")

        self.enabled.SetValue(default_enabled)

        self.position.Disable()

    def setPosition(self, angle):
        self.position.SetValue(int(angle * 100))

    def getPosition(self):
        return self.position.GetValue() / 100.0


class controllerGUI(wx.Frame):
    TIMER_ID = 100

    def __init__(self, parent, debug=False, default_enabled=False):
        wx.Frame.__init__(self, parent, -1, "Controller GUI",
                          style=wx.DEFAULT_FRAME_STYLE | (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))
        # sizer = wx.GridBagSizer(0,0)
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.default_enabled = default_enabled
        self.ready = False
        # Move Servos
        # servo = wx.StaticBox(self, -1, 'Move Servos')
        # servo.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        # servoBox = wx.StaticBoxSizer(servo,orient=wx.VERTICAL )
        servoSizer = wx.GridBagSizer(5, 5)
        # servoSizer = wx.GridSizer(30,5,1,1)
        # servoSizer = wx.BoxSizer(wx.VERTICAL)

        # joint_defaults = getJointsFromURDF()
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

        self.dynamixels = dict()

        for name in dynamixels_names['names']:
            # print name
            self.dynamixels[name] = rospy.get_param(name, dict())

        self.servos = dict()
        self.servos_pos = dict()
        self.publishers = dict()
        self.subscribers = dict()
        self.services = dict()
        # self.relaxers = list()

        # joints = rospy.get_param('/arbotix/joints', dict())
        # create sliders and publishers
        self.labels = dict()
        self.labels_objs = dict()
        self.text_entry = dict()

        self.id_name = dict()
        i = 0
        for name in sorted(self.dynamixels.keys()):
            # print n,dynamixels[n]['motor']['max']#['joint_name']

        # for name in sorted(joints.keys()):
            # pull angles
            # min_angle, max_angle = getJointLimits(name, joint_defaults)
            init_angle_raw = self.dynamixels[name]['motor']['init']
            # print params
            min_angle = (self.dynamixels[name]['motor']['min'] - init_angle_raw) / self.params[
                int(self.dynamixels[name]['motor']['id'])]['encoder_ticks_per_radian']
            max_angle = (self.dynamixels[name]['motor']['max'] - init_angle_raw) / self.params[
                int(self.dynamixels[name]['motor']['id'])]['encoder_ticks_per_radian']

            # in degrees?
            # min_angle = degrees(min_angle)
            # max_angle = degrees(max_angle)
            # min_angle = dynamixels[name]['motor']['min']/params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']
            # max_angle =
            # dynamixels[name]['motor']['max']/params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']
            init_angle = self.dynamixels[name]['motor']['init'] / self.params[
                self.dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']
            self.id_name[self.dynamixels[name]['motor']['id']] = name
            # init_angle = degrees(init_angle)

            # init_angle=0.0

            # print
            # min_angle,max_angle,init_angle,dynamixels[name]['motor']['min'],dynamixels[name]['motor']['max'],params[dynamixels[name]['motor']['id']]['encoder_ticks_per_radian']

            # create publisher
            self.publishers[self.dynamixels[name]['motor']['id']] = rospy.Publisher(
                name + '/command', Float64)

            # init?
            # self.publishers[self.dynamixels[name][
            #     'motor']['id']].publish(Float64(0.0))
            # time.sleep(0.01)
            # if rospy.get_param('/arbotix/joints/'+name+'/type','dynamixel') == 'dynamixel':
            #     self.relaxers.append(rospy.ServiceProxy(name+'/relax', Relax))
            # else:
            #     self.relaxers.append(None)

            # create subscriber (should not work in "fast" mode but thanks to
            # ROS it still works)
            self.subscribers[self.dynamixels[name]['motor']['id']] = rospy.Subscriber(
                name + '/state', JointState, self.stateCb)

            # create service proxy
            self.services[self.dynamixels[name]['motor']['id']] = rospy.ServiceProxy(
                name + '/set_torque_limit', SetTorqueLimit)

            self.services[
                self.dynamixels[name]['motor']['id']](INIT_TORQUE)

            # if not self.default_enabled:
            #     self.services[
            #         self.dynamixels[name]['motor']['id']](INIT_TORQUE)
            # else:
            #     self.services[
            #         self.dynamixels[name]['motor']['id']](ON_TORQUE)

            # create slider
            # s = servoSlider(self, min_angle, max_angle, name, i)

            if min_angle < max_angle:

                s = servoSlider(self, min_angle, max_angle, self.dynamixels[
                    name]['joint_name'], self.dynamixels[name]['motor']['id'], default_enabled)
            else:

                s = servoSlider(self, max_angle, min_angle, self.dynamixels[
                    name]['joint_name'], self.dynamixels[name]['motor']['id'], default_enabled)

            s.setPosition(init_angle)
            # servoSizer.Add(s.enabled,(i,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
            # servoSizer.Add(s.position,(i,1),
            # wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)

            servoSizer.Add(
                s.enabled, (i, 0), wx.DefaultSpan, wx.ALIGN_CENTER_VERTICAL)
            servoSizer.Add(
                s.position, (i, 1), wx.DefaultSpan, wx.ALIGN_CENTER_VERTICAL)

            self.labels_objs[self.dynamixels[name]['motor']['id']] = wx.StaticText(
                self, label=str(s.getPosition()))

            self.text_entry[self.dynamixels[name]['motor']['id']] = wx.TextCtrl(
                self, self.dynamixels[name]['motor']['id'], size=(-1, -1), style=wx.TE_PROCESS_ENTER)

            self.labels[self.dynamixels[name]['motor']['id']] = ""

            # servoSizer.Add(self.labels_objs[dynamixels[name]['motor']['id']], (i,2), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
            # servoSizer.Add(self.text_entry[dynamixels[name]['motor']['id']],
            # (i,4), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)

            # servoSizer.Add(self.labels_objs[dynamixels[name]['motor']['id']], (i,2), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
            # servoSizer.Add(self.text_entry[dynamixels[name]['motor']['id']],
            # (i,4), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)

            servoSizer.Add(self.labels_objs[self.dynamixels[name]['motor']['id']], (
                i, 2), wx.DefaultSpan, wx.ALIGN_CENTER_VERTICAL)

            servoSizer.Add(self.text_entry[self.dynamixels[name]['motor']['id']], (
                i, 4), wx.DefaultSpan, wx.ALIGN_CENTER_VERTICAL)

            self.servos[self.dynamixels[name]['motor']['id']] = s
            self.servos_pos[self.dynamixels[name]['motor']['id']] = init_angle

            # self.publishers[self.dynamixels[name][
            #     'motor']['id']].publish(Float64(0.0))
            # time.sleep(0.01)
            # if not self.default_enabled:
            #     self.services[
            #         self.dynamixels[name]['motor']['id']](INIT_TORQUE)
            # else:
            #     self.services[
            #         self.dynamixels[name]['motor']['id']](ON_TORQUE)

            i += 1

        # add everything
        # servoBox.Add(servoSizer)
        # sizer.Add(servoBox, (0,1), wx.GBSpan(1,1),
        # wx.EXPAND|wx.TOP|wx.BOTTOM|wx.RIGHT,5)
        # sizer.Add(servoSizer, (0,1), wx.GBSpan(1,1),
        # wx.EXPAND|wx.TOP|wx.BOTTOM|wx.RIGHT,5)
        # sizer.Add(servoSizer, (1,1),wx.DefaultSpan, wx.EXPAND|wx.ALL,5)
        sizer.Add(servoSizer, 1, wx.EXPAND | wx.ALL, 5)

        # sizer.Add(servoBox, (0,1), wx.DefaultSpan, wx.EXPAND|wx.TOP|wx.BOTTOM|wx.RIGHT,5)
        # ICI
        # panel=wx.Panel(self)

        self.Bind(wx.EVT_CHECKBOX, self.enableSliders)

        self.Bind(wx.EVT_TEXT_ENTER, self.onTextEnter)
        # self.Bind(wx.EVT_SET_CURSOR, self.onTextInsert)
        # self.Bind(wx.EVT_COMMAND_LEFT_CLICK, self.onTextInsert)

        # now we can subscribe

        #
        # rospy.Subscriber('joint_states', JointState, self.stateCb)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(TIMER_DT)
        wx.EVT_CLOSE(self, self.onClose)
        wx.EVT_TIMER(self, self.TIMER_ID, self.onTimer)

        # bind the panel to the paint event
        # wx.EVT_PAINT(self, self.onPaint)
        # self.dirty = 1
        # self.onPaint()

        self.ready = True
        self.SetSizerAndFit(sizer)
        self.Show(True)

    def onClose(self, event):
        rospy.signal_shutdown("Closing")
        self.timer.Stop()
        self.Destroy()

    def enableSliders(self, event):
        servo = event.GetId()

        # print 'event ID', servo
        # print event
        if event.IsChecked():
            self.servos[servo].position.Enable()

            self.services[servo](ON_TORQUE)
            print 'ID %d torque %f' % (servo, ON_TORQUE)
            # self.servos[servo].position.Enable()
            # ici torque on

        else:
            # self.servos[servo].position.Disable()
            self.servos[servo].position.Disable()
            # ici torque off
            self.services[servo](OFF_TORQUE)

            print 'ID %d torque off' % (servo)
            # if self.relaxers[servo]:
            #     self.relaxers[servo]()

    def rad_to_raw(self, motor_id, angle):
        """ angle is in radians """
        # print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped),
        # angle, initial_position_raw)
        name = self.id_name[motor_id]
        initial_position_raw = self.dynamixels[name]['motor']['init']

        angle_raw = angle * \
            self.params[int(self.dynamixels[name]['motor']['id'])][
                'encoder_ticks_per_radian']

        flipped = self.dynamixels[name]['motor'][
            'min'] > self.dynamixels[name]['motor']['max']

        # print 'angle = %f, val = %d' % (math.degrees(angle),
        # int(round(initial_position_raw - angle_raw if flipped else
        # initial_position_raw + angle_raw)))
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

    def stateCb(self, msg):

        self.servos_pos[msg.motor_ids[0]] = msg.current_pos
        # deg
        # self.servos_pos[msg.motor_ids[0]] = degrees(msg.current_pos)

        # self.labels[msg.motor_ids[0]].SetLabel('%.3f'%(msg.current_pos))
        self.labels[msg.motor_ids[0]] = "%.3f (%d)" % (
            msg.current_pos, self.rad_to_raw(msg.motor_ids[0], msg.current_pos))
        # self.labels[msg.motor_ids[0]] = "%.3f" % (degrees(msg.current_pos))

            # self.labels[msg.motor_ids[0]].SetLabel('%.3f'%(msg.current_pos))
            # print '%.3f'%(msg.current_pos)
        # for servo in self.servos:
        #     if not servo.enabled.IsChecked():
        #         try:
        #             idx = msg.name.index(servo.name)
        #             servo.setPosition(msg.position[idx])
        #         except:
        #             pass
    def onTextEnter(self, event):
        s = event.GetId()

        if self.servos[s].enabled.IsChecked():
            self.servos[s].setPosition(
                radians(float(self.text_entry[s].GetValue())))

    def onTimer(self, event=None):
        d = Float64()
        # send joint updates
        # for s,p in zip(self.servos,self.publishers):
        #     if s.enabled.IsChecked():
        #         d = Float64()
        #         d.data = s.getPosition()

                #
                # p.publish(d)

        # for s in self.servos.keys():
        #     if self.servos[s].enabled.IsChecked():
        #         d = self.servos[s].getPosition()
        #         if self.labels_objs[s].GetLabel() != str(d):
        #             self.labels_objs[s].SetLabel(str(d))
        #         self.publishers[s].publish(d)
        # print s
        #     else:
        #         d = self.servos[s].getPosition()
        #         if self.labels_objs[s].GetLabel() != self.labels[s]:
        #             self.labels_objs[s].SetLabel(self.labels[s])

        #         self.servos[s].setPosition(self.servos_pos[s])

        # deg

        if self.ready:

            for s in self.servos.keys():
                if self.servos[s].enabled.IsChecked():
                    d = self.servos[s].getPosition()

                    if self.labels_objs[s].GetLabel() != str(d):
                        self.labels_objs[s].SetLabel(str(d))
                    self.publishers[s].publish(d)
                    # print s, d
                else:
                    # d = self.servos[s].getPosition()

                    if self.labels_objs[s].GetLabel() != self.labels[s]:
                        self.labels_objs[s].SetLabel(self.labels[s])

                    self.servos[s].setPosition(self.servos_pos[s])


if __name__ == '__main__':
    # initialize GUI

    parser = argparse.ArgumentParser(description='Simple robot gui')
    parser.add_argument('--torque', default="off", type=str,
                        help='Init motor torque: on or off')

    args = parser.parse_args()
    rospy.init_node('controllerGUI')
    torque = False
    if args.torque == "on":
        torque = True

    app = wx.PySimpleApp()
    frame = controllerGUI(None, True, torque)
    app.MainLoop()
