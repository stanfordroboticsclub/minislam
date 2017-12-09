#!/usr/bin/env python
from __future__ import division

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import tf

from threading import Timer


class Particle:

    def __init__(self, x, y, rot):
        self.x = x
        self.y = y
        self.rot = rot

    def spread_out(self):
        pass

    def simulate_lidar(self,map):
        pass

class Map:

    def __init__(self):
        self.x_size_m = 100
        self.y_size_m = 100
        self.resolution = 0.05

        self.x_size_g = int(self.x_size_m/self.resolution)
        self.y_size_g = int(self.y_size_m/self.resolution)

        self.map = [-1] * self.y_size_g*self.x_size_m

        self.map_topic = 'map_test'
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=10)

    def __getitem__(self, key):
        return self.map[ key[1] * self.x_size_g + key[0] ]

    def __setitem__(self, key, value):
        self.map[ key[1] * self.x_size_g + key[0] ] = value


    def publish_map(self):
        self.time = rospy.Time.now() 

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.time 
        map_msg.header.frame_id = self.world_frame

        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.y_size_g
        map_msg.info.height = self.x_size_g

        map_msg.info.origin.position.x = - self.x_size_m/2
        map_msg.info.origin.position.y = - self.y_size_m/2
        map_msg.info.origin.position.z = 0
        
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 0

        map_msg.data = [item for sublist in self.map for item in sublist]

        self.map_pub.publish(map_msg)

        map_timer = Timer(0.1, self.publish_map, ())
        map_timer.start()

        

class ParticleFilter:
    def __init__(self):
        self.numParticles = 100

        self.world_frame = 'map'

        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.callback)
        #initalise particles

        self.particles = []

        self.map = Map()



    def update_odometery(self):
        pass
    
    def resample_from_lidar(self):
        pass

    def update_map(self):
        pass

    def add_noise(self):
        pass

    def get_mean_position(self):
        pass

    def callback(self,data):

        print data.ranges

def main():

    rospy.init_node('minislam')

    pf = ParticleFilter()

    pf.publish_map()

    rospy.spin()

if __name__ == '__main__':
    main()


