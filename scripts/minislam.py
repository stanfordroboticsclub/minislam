#!/usr/bin/env python
from __future__ import division

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs import LaserScan
from nav_msgs.msg import Odometry
import tf


class Particle:

    def __init__(self, x, y, rot):
        self.x = x
        self.y = y
        self.rot = rot

    def spread_out(self):
        pass

class ParticleFilter:
    def __init__(self, numParticles):
        self.numParticles = numParticles

        #initalise particles

        self.particles = []


    def update_odometery(self):
        pass


    def get_mean_position(self):
        pass



def main():

    rospy.init_node('minislam')
    rospy.Subscriber("turtle1/cmd_vel", LaserScan, callback)

    pf = ParticleFilter()

    rospy.spin()

if __name__ == '__main__':
    main()


