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

    def simulate_lidar(curr_map, thetas, range_max): #num_theta is length of lidar array
        THRESHOLD = 0
        thetas = thetas + self.rot
        drange = np.linspace(0, curr_map.size_m/2, curr_map.resolution/2)
        for theta in thetas:
            dx = math.cos(theta)
            dy = math.sin(theta)
            min_val = range_max
            
            for d in drange:
                dxi = d * dx
                dyi = d * dy
                
                if abs(dxi + self.x) < curr_map.size_m/2 and abs(dyi + self.y) < curr_map.size_m/2
                and curr_map.map[self.x + dxi, self.y + dyi] > THRESHOLD:
                    disp = math.sqrt(dxi * dxi + dyi * dyi)
                    if disp < min_val:
                        min_val = disp
        return min_val

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
        print "map published"
        

class ParticleFilter:
    def __init__(self):
        self.numParticles = 100
        self.world_frame = 'map'
        self.map_topic = 'map_test'

        #initialise particles
        self.particles = [ Particle(0,0,0) for _ in range(self.numParticles) ] 
        self.weights = None

        self.map = Map(self.map_topic,self.world_frame)
        self.laser_data = None


        self.map[0,0] = 40
        self.map[0,0.1] = 50
        self.map[0,-0.1] = 30

        rospy.Subscriber("scan", LaserScan, self.scan_callback)


    def update_odometery(self):
        pass
    
    def get_particle_weights(self):
        for particle in self.particles:
            sim_scan = particle.simulate_lidar(self.map)
            

    def resample_particles(self):
        # self.particles = list(
        inds = np.random.choice(np.arange(self.numParticles), 
                             size=self.numParticles, replace=True, p=self.weights)

        temp = [ self.particles[i] for i in inds]
        self.particles = temp

    def update_map(self):
        for dist,angle in zip(self.laser_data, np.arange(self.angle_min,self.angle_max,self.angle_increment)):

            if dist > self.range_min and dist < self.range_max and dist != float('inf'):
                for d in np.arange(0, dist, self.map.resolution/2):
                    ind =( self.x_mean + d*math.cos(self.rot_mean+ angle), self.y_mean + d*math.sin(self.rot_mean + angle) )

                    if self.map[ind] == -1:
                        self.map[ind] = 100
                    else:
                        self.map[ind] *= 0.5
                
                ind =( self.x_mean + dist*math.cos(self.rot_mean + angle), self.y_mean + dist*math.sin(self.rot_mean + angle) )
                if self.map[ind] == -1:
                    self.map[ind] = 0
                else:
                    self.map[ind] = 100 - (0.5 * (100 - self.map[ind]))


    def add_noise(self):
        for particle in self.particles:
            particle.spread_out()

    def get_mean_position(self):
        self.x_mean = 0
        self.y_mean = 0
        self.rot_mean = 0
        for particle in self.particles:
            self.x_mean += particle.x
            self.y_mean += particle.y
            self.rot_mean += particle.rot

        self.x_mean /= float(self.numParticles)
        self.y_mean /= float(self.numParticles)
        self.rot_mean /= float(self.numParticles)

    def scan_callback(self,data):
        print 'got scan'
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.time_increment = data.time_increment
        self.scan_time = data.scan_time
        self.range_min = data.range_min
        self.range_max = data.range_max

        if self.laser_data==None:
            print 'start'
            self.laser_data = data.ranges
            self.run()
        else:
            self.laser_data = data.ranges

    def run(self):

        self.update_odometery() 
        # self.add_noise()
        # self.resample_particles()
        self.get_mean_position()
        self.update_map()

        self.map.publish_map()

        main_timer = Timer(0.5, self.run, ())
        main_timer.start()

def main():

    rospy.init_node('minislam')

    pf = ParticleFilter()

    rospy.spin()

if __name__ == '__main__':
    main()


