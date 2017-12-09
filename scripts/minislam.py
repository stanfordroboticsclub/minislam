#!/usr/bin/env python
from __future__ import division

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import tf

from threading import Timer
import random
import math
import numpy as np


class Particle:

    def __init__(self, x, y, rot):
        self.x = x
        self.y = y
        self.rot = rot

    def spread_out(self):
        self.x += random.gauss(0, 0.3)
        self.y += random.gauss(0, 0.3)
        self.rot += random.gauss(0, math.pi/6)

    def simulate_lidar(self,map):
        pass

class Map:

    def __init__(self,map_topic, world_frame):
        self.x_size_m = 100
        self.y_size_m = 100
        self.resolution = 0.05

        self.x_size_g = int(self.x_size_m/self.resolution)
        self.y_size_g = int(self.y_size_m/self.resolution)

        self.map = [-1] * self.y_size_g*self.x_size_g

        self.map_topic = map_topic
        self.world_frame = world_frame
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=10)

    def __getitem__(self, key):
        x = int((key[0] + self.x_size_m/2)/self.resolution)
        y = int((key[1] + self.y_size_m/2)/self.resolution)
        return self.map[ y * self.x_size_g + x ]

    def __setitem__(self, key, value):
        x = int((key[0] + self.x_size_m/2)/self.resolution)
        y = int((key[1] + self.y_size_m/2)/self.resolution)
        self.map[ y * self.x_size_g + x ] = value

    def get_raw(self, x , y):
        return self.map[ y * self.x_size_g + x ]

    def set_raw(self, x , y, value):
        self.map[ y * self.x_size_g + x ] = value


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

        map_msg.data = self.map

        self.map_pub.publish(map_msg)
        

class ParticleFilter:
    def __init__(self):
        self.numParticles = 100
        self.world_frame = 'map'
        self.map_topic = 'map_test'

        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.callback)

        #initialise particles
        self.particles = [ Particle(0,0,0) for _ in range(self.numParticles) ] 
        self.weights = None

        self.map = Map(self.map_topic,self.world_frame)
        self.laser_data = None


        self.map[0,0] = 40
        self.map[0,0.1] = 50
        self.map[0,-0.1] = 30


    def update_odometery(self):
        pass
    
    def get_particle_weights(self):
        pass

    def resample_particles(self):
        self.particles = list(
            np.random.choice(np.array(self.particles), 
                             size=self.numParticles, replace=True, p=self.weights))

    def update_map(self):
        pass

    def add_noise(self):
        for particle in self.particles:
            particle.spread_out()

    def get_mean_position(self):
        x_mean = 0
        y_mean = 0
        rot_mean = 0
        for particle in self.particles:
            x_mean += particle.x
            y_mean += particle.y
            rot_mean += particle.rot

        x_mean /= float(self.numParticles)
        y_mean /= float(self.numParticles)
        rot_mean /= float(self.numParticles)

    def callback(self,data):
        if self.laser_data==None:
            self.laser_data = data.ranges
            # self.run()
        else:
            self.laser_data = data.ranges

        self.map.publish_map()

    def run(self):
        self.update_odometery() 
        self.add_noise()
        self.resample_particles()
        self.update_map()

        self.get_mean_position()
        self.map.publish_map()

def main():

    rospy.init_node('minislam')

    pf = ParticleFilter()

    pf.map.publish_map()

    rospy.spin()

if __name__ == '__main__':
    main()


