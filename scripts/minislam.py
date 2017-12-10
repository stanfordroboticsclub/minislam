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


"""class Particle:
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
"""

class Map:
    """ Map object
    setting/getting is in meters
    main field of interest is self.grid
    """
    def __init__(self, map_topic, world_frame, m=100, n=100, res=0.05):
        self.x_size_m = m
        self.y_size_m = n
        self.resolution = res
        self.x_size_g = int(self.x_size_m/self.resolution)
        self.y_size_g = int(self.y_size_m/self.resolution)

        self.grid = np.randint(0, 2, size=(self.y_size_g, self.x_size_g)) #changed this to row

        self.map_topic = map_topic
        self.world_frame = world_frame
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=10)

    def __getitem__(self, key):
        x = int((key[0] + self.x_size_m/2)/self.resolution)
        y = int((key[1] + self.y_size_m/2)/self.resolution)
        return self.grid[y * self.x_size_g + x]

    def __setitem__(self, key, value):
        x = int((key[0] + self.x_size_m/2)/self.resolution)
        y = int((key[1] + self.y_size_m/2)/self.resolution)
        self.grid[ y * self.x_size_g + x ] = value

    def get_raw(self, x , y):
        return self.grid[ y * self.x_size_g + x ]

    def set_raw(self, x , y, value):
        self.grid[ y * self.x_size_g + x ] = value

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

        map_msg.data = self.grid.reshape(self.x_size_g * self.y_size_g)

        self.map_pub.publish(map_msg)
        

class ParticleFilter:
    def __init__(self, num_p = 100):
        self.num_particles = num_p
        self.world_frame = 'map'
        self.map_topic = 'map_test'

        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.callback)

        #initialise particles
        self.particles = np.zeros((2, num_p))
        self.rotations = np.rand(0, 2 * math.pi, size=num_p)
        self.weights = np.empty(self.num_particles)

        self.map= Map(self.map_topic,self.world_frame)
        self.laser_data = None

        self.map.grid[0,0] = 40
        self.map.grid[0,0.1] = 50
        self.map.grid[0,-0.1] = 30


    def update_odometry(self):
        pass
    
    def get_particle_weights(self):
        num_theta = len(data.ranges)
        thetas = np.linspace(0, 2 * math.pi, num=num_theta)
        th, rot = np.meshgrid(thetas, self.rotations)
        #TODO: check that each row corresponds to a particle, each column to an angle
        angles = (th + rot)  #might be greater than 2pi
        
        x = np.linspace(-self.map.x_size_m/2, self.map.x_size_m/2, self.laser_data.size[0])
        for idx, theta in np.ndenumerate(angles):
            min_val = self.maxrange
            for x_j in x:
                tan_th = math.tan(theta)
                y = tan_th * x + self.particles[1][idx[0]] - tan_th * self.particles[0][idx[0]]
            [idx[1]]


        for i in range(self.num_particles):
            self.weights[i] = np.linalg.norm(self.laser_data - sim[i, :])

    def resample_particles(self):#TODO: redo
        self.particles = list(
            np.random.choice(np.array(self.particles), 
                             size=self.numParticles, replace=True, p=self.weights))

    def update_map(self):
        #TODO update this

        
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
        self.maxrange = data.range_max

        if self.laser_data==None:
            print('sad') #TODO remove this or fix
            # self.run()

        self.laser_data = np.array(data.ranges)
        
        # makes sure all values are in range
        self.laser_data[self.laser_data < data.range_min] = data.range_min
        self.laser_data[self.laser_data > data.range_max] = data.range_max

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


