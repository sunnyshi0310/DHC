#!/usr/bin/env python

import rospy
import argparse
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as q2e
from crazyflie_driver.msg import GenericLogData


class CrazyflieParser:
    def __init__(self, index, initialPosition):
        parser = argparse.ArgumentParser()
        parser.add_argument("--sim", help="Run using simulation", action="store_true")
        parser.add_argument("--vis", help="(sim only) Visualization backend [mpl]", choices=['mpl', 'vispy'], default="mpl")
        parser.add_argument("--dt", help="(sim only) dt [0.1s]", type=float, default=0.1)
        parser.add_argument("--writecsv", help="(sim only) Enable CSV output", action="store_true")
        args, unknown = parser.parse_known_args()

        if args.sim:
            from crazyflieSim import Crazyflie, TimeHelper
            self.timeHelper = TimeHelper(args.vis, args.dt, args.writecsv)
            cf = Crazyflie(index, initialPosition, self.timeHelper)
            self.crazyflies = []
            self.crazyflies.append(cf)
            self.timeHelper.crazyflies = self.crazyflies
        else:
            from crazyflie import Crazyflie, TimeHelper
            self.timeHelper = TimeHelper()
            self.trans = []
            self.rot = []
            rospy.init_node('CrazyflieParser')
            rospy.Subscriber("/cf1/odom",Odometry,self.callback)
            rospy.Subscriber("/cf1/motor", GenericLogData,self.back)
            cf = Crazyflie(index, initialPosition, self.timeHelper)
            self.crazyflies = []
            self.crazyflies.append(cf)


    def callback(self,data):
        self.trans = np.array([data.pose.pose.position.x,data.pose.pose.position.z])
        self.xy = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        self.ori = np.array([orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w])
        rotation = np.array(q2e(orientation_list))
        self.rot = rotation[1]
        self.yaw = rotation[2]
        self.roll = rotation[0]

    def back(self,data):
        value = np.array(data.values)
        self.mot = value
