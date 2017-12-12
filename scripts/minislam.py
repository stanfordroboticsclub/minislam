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
from collections import Counter


class Particle:

    def __init__(self, x, y, rot):
        self.x = x
        self.y = y
        self.rot = rot

    def copy(self):
        return Particle(self.x, self.y ,self.rot)

    def spread_out(self,std):
        drive = random.gauss(0, std)
        side = random.gauss(0, std*0.5) 
        self.x += math.cos(self.rot) * drive + math.sin(self.rot) * side
        self.y += math.sin(self.rot) * drive + math.cos(self.rot) * side

        # self.x += random.gauss(0, std)
        # self.y += random.gauss(0, std) 
        self.rot += random.gauss(0, 4*std)

        # self.rot += random.gauss(0, math.pi/6)

    def get_lidar_prob(self,map, angles,ranges):

        log_prob = 0
        good_values = 0
        for angle,dist in zip(angles , ranges):
            if dist == float('inf'):
                continue
            x_impact = self.x + dist * math.cos(self.rot + angle)
            y_impact = self.y + dist * math.sin(self.rot + angle)

            value = map[x_impact, y_impact]

            if value == -1:
                continue
            good_values += 1

            try:
                # log_prob += math.log( value/float(100)+1 )
                log_prob +=  value/float(100) 
            except ValueError:
                print value
                raise

        if good_values ==0:
            return 0.00001
        return (log_prob/good_values)


    def simulate_lidar(self,curr_map, thetas, range_max): #num_theta is length of lidar array
        retval = np.empty(thetas.shape[0])
        drange = np.arange(0, range_max, curr_map.resolution/2)



        for x in range(thetas.shape[0]):
            theta = thetas[x] + self.rot
            dx = math.cos(theta)
            dy = math.sin(theta)
            
            disp = float('inf')

            for d in drange:
                dxi = d * dx
                dyi = d * dy
                
                if abs(dxi + self.x) < curr_map.x_size_m/2 and abs(dyi + self.y) < curr_map.y_size_m/2 \
                                and curr_map[self.x + dxi, self.y + dyi] > THRESHOLD:
                    disp = math.sqrt(dxi * dxi + dyi * dyi)
                    break

            retval[x] = disp
        return retval

class Map:

    def __init__(self,map_topic, world_frame):
        self.x_size_m = 40
        self.y_size_m = 40
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

        map_msg.data = self.map[:]

        self.map_pub.publish(map_msg)
        print "map published"
        

class ParticleFilter:
    def __init__(self):
        self.numParticles = 100
        self.world_frame = 'map'
        self.map_topic = 'map_test'

        self.trans = tf2_ros.TransformBroadcaster()

        #initialise particles
        self.particles = [ Particle(0,0,0) for _ in range(self.numParticles) ] 
        self.weights = None

        self.map = Map(self.map_topic,self.world_frame)
        self.laser_data = None


        rospy.Subscriber("scan", LaserScan, self.scan_callback)

        self.x_mean = 0
        self.y_mean = 0
        self.rot_mean = 0


    def update_odometery(self):
        pass
    
    def get_particle_weights(self):
        request_angles = np.arange(self.angle_min, self.angle_max+self.angle_increment, self.angle_increment)
        freeze_laser = self.laser_data

        # weights = np.exp (np.array( [   particle.get_lidar_prob(self.map, request_angles, freeze_laser) for particle in self.particles]))
        self.weights = np.array( [   particle.get_lidar_prob(self.map, request_angles, freeze_laser) for particle in self.particles])
        self.weights = np.abs(self.weights)
        # print weights

        # tot = np.sum(weights)
        # print 
        if np.sum(self.weights) == 0:
            weights = np.array( [   1 for particle in self.particles])
            # tot = np.sum(weights)
        # self.weights = weights / tot

            


    def resample_particles(self):


        # for i,p in enumerate(self.particles):
        #     if math.fabs(p.x - self.x_mean) > 0.1 or math.fabs(p.y - self.y_mean) > 0.1 or math.fabs(p.rot - self.rot_mean) > 0.9:
        #         self.weights[i] = 0

        self.weights = self.weights / np.sum(self.weights)


        # inds = np.random.choice(np.arange(self.numParticles), size=self.numParticles, replace=True, p=self.weights)

        inds =[ max( (self.weights[i], i ) for i in range(self.numParticles))[1] ] * self.numParticles
        # print Counter(inds)

        print 'x', np.std ( [ p.x for p in self.particles ] )
        print 'y', np.std ( [ p.y for p in self.particles ] )
        print 'rot', np.std ( [ p.rot for p in self.particles ] )

        temp = [ self.particles[i].copy() for i in inds]
        self.particles = temp

    def update_map(self):

        ratio = 0.95
        for dist,angle in zip(self.laser_data, np.arange(self.angle_min,self.angle_max,self.angle_increment)):

            if dist > self.range_min and dist < self.range_max and dist != float('inf'):
                for d in np.arange(0, dist, self.map.resolution/2):
                    ind =( self.x_mean + d*math.cos(self.rot_mean+ angle), self.y_mean + d*math.sin(self.rot_mean + angle) )
                    # if self.map[ind] == -1:
                    #     self.map[ind] = 0
                    # else:
                    #     self.map[ind] *= 0.2

                    self.map[ind] *= ratio

                feature_size =3*self.map.resolution 
                for d in np.arange(-feature_size, feature_size  , self.map.resolution/4):
                    ind =( self.x_mean + (dist+d)*math.cos(self.rot_mean+ angle), self.y_mean + (dist+d)*math.sin(self.rot_mean + angle) )
                    # if self.map[ind] == -1:
                    #     self.map[ind] = 100
                    # else:
                    #     self.map[ind] = 100 - (0.2 * (100 - self.map[ind]))
                    
                    if (feature_size - math.fabs(d)) / feature_size <0:
                        continue

                    target =100 * (feature_size - math.fabs(d)) / feature_size 
                    self.map[ind] =  (1-ratio) * target + ratio * self.map[ind]
                


    def add_noise(self,std):
        for particle in self.particles:
            particle.spread_out(std)

    def get_mean_position(self):
        self.x_mean = 0
        self.y_mean = 0
        # self.rot_mean = 0

        rot_x = 0
        rot_y = 0
        for particle in self.particles:
            self.x_mean += particle.x
            self.y_mean += particle.y
            # self.rot_mean += particle.rot
            rot_x += math.cos(particle.rot)
            rot_y += math.sin(particle.rot)

        self.x_mean /= float(self.numParticles)
        self.y_mean /= float(self.numParticles)
        # self.rot_mean /= float(self.numParticles)

        rot_y /=float(self.numParticles) 
        rot_x /=float(self.numParticles) 

        print 'rots',rot_y,rot_x

        self.rot_mean = math.atan2(rot_y, rot_x)

        print self.x_mean, self.y_mean,self.rot_mean

    def scan_callback(self,data):
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

    def send_trans(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.rot_mean)
        self.quat = Quaternion(*q)

        transform = TransformStamped()
        transform.header.stamp = self.map.time
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = 'laser'

        transform.transform.translation.x = self.x_mean
        transform.transform.translation.y = self.y_mean
        transform.transform.translation.z = 0.0
        transform.transform.rotation = self.quat

        self.trans.sendTransform(transform)

    def run(self):

        self.update_odometery() 
        print 'add noise'

        # self.add_noise(0.1)
        # self.get_particle_weights()
        # self.resample_particles()

        self.add_noise(0.15)
        self.get_particle_weights()
        self.resample_particles()

        self.add_noise(0.05)
        self.get_particle_weights()
        self.resample_particles()

        self.add_noise(0.01)
        self.get_particle_weights()
        self.resample_particles()

        self.get_mean_position()
        self.update_map()

        self.map.publish_map()

        self.send_trans()

        main_timer = Timer(0.01, self.run, ())
        main_timer.start()

def main():

    rospy.init_node('minislam')

    pf = ParticleFilter()

    rospy.spin()
    print 'here'




if __name__ == '__main__':
    main()


