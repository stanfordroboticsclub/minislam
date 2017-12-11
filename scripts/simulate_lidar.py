import gi
import numpy as np
import math
from gi.repository import Gtk
from scipy.stats import norm

class Grid:
    def __init__(self, m, n, num_theta, max_r):
        self.m = m
        self.n = n
        self.max_range = max_r
        self.grid = np.random.choice([0, 1], p = [0.9, 0.1], size=(m, n))
        self.loc = (np.random.randint(0, m), np.random.randint(0, n))
        self.rot = np.random.uniform(0, 2 * math.pi)
        while (self.grid[self.loc] != 0):
            self.loc = (np.random.randint(0, m), np.random.randint(0, n))
        self.thetas = np.linspace(0, 2 * math.pi, num_theta)

    def observe_lidar(self, particle):
        angles = self.thetas + particle[2]
        distances = np.empty(angles.shape[0])
        for i in range(angles.shape[0]):
            theta = angles[i]
            disp = self.max_range
            x = particle[0]
            y = particle[1]
            dx = math.cos(theta)
            dy = math.sin(theta)
            while (x >= 0 and x <= self.m - 1 and y >= 0 and y <= self.n - 1):
                if (self.grid[x, y] > 0):
                    dx = x - particle[0]
                    dy = y - particle[1]
                    disp = dx * dx + dy * dy
                    break
                
                x += dx
                y += dy

            distances[i] = disp #+ np.random.normal(scale = 0.8)
        return distances

    def get_observation(self):
        dist = self.observe_lidar((self.loc[0], self.loc[1], self.rot))
        noise = np.random.normal(scale = 0.08, size = dist.shape[0])
        return dist + noise

    def unoccupied(self):
        inds = np.arange(self.m*self.n).reshape((self.m, self.n))
        return inds[self.grid != 1]

    def move(self):
        self.rot = np.random.normal(loc=self.rot, scale=math.pi/8)
        r = np.random.randint(3, 10)
        dx = math.cos(self.rot)
        dy = math.sin(self.rot)
        steps = 0
        new_loc = self.loc
        while (steps < r and new_loc[0] >= 0 and new_loc[0] <= self.m - 1 and new_loc[1] >= 0 and
               new_loc[1] <= self.n - 1 and self.grid[new_loc] != 1):
            self.loc = new_loc
            new_loc = (new_loc[0] + dx, new_loc[1] + dy)
            steps += 1
            
        self.grid[self.loc] = 0
        self.loc = new_loc
        self.grid[self.loc] = -1

class ParticleFilter:
    def __init__(self, m, n, num_particles, num_theta, max_r):
        self.map = Grid(m, n, num_theta, max_r)
        self.num_p = num_particles
        self.weights = 1.0 / num_particles * np.ones(num_particles)
        possibilities = self.map.unoccupied()
        np.random.shuffle(possibilities)
        indices = possibilities[0:num_particles]
        
        self.particles = np.empty((3, num_particles))
        self.particles[0, :] = indices / self.map.n
        self.particles[1, :] = indices % self.map.n
        self.particles[2, :] = np.random.uniform(0, 2 * math.pi, num_particles)

        self.calc_weights()
        self.resample_particles()

    def calc_weights(self):
        dists = self.map.get_observation()
        for p in range(self.num_p):
            p_dists = self.map.observe_lidar(self.particles[:, p])
            self.weights[p] = np.linalg.norm(dists - p_dists)

        self.weights[p] = norm(0, 100).pdf(self.weights[p])
        #max_w = np.amax(self.weights)
        #self.weights = max_w - self.weights
        tot = np.sum(self.weights)
        self.weights = self.weights / tot
        print(self.weights)

    def resample_particles(self):
        new_ind = np.random.choice(np.arange(self.num_p), p = self.weights, size = self.num_p)
        new_part = np.empty((3, self.num_p))
        for i in new_ind:
            new_part[:, i] = self.particles[:, new_ind[i]]

        noise = np.random.normal(scale = 2, size = ((3, self.num_p)))
        noise[2, :] = np.random.normal(scale = math.pi / 8.0, size = self.num_p)

        self.particles = new_part + noise

    def step(self):
        #self.map.move()
        self.calc_weights()
        self.resample_particles()
        corr = np.empty((2, 1))
        corr[0, 0] = self.map.loc[0]
        corr[1, 0] = self.map.loc[1]
        diff = self.particles[0:2, :] - corr
        min_value = 1000000
        for x in range(diff.shape[1]):
            if np.linalg.norm(diff) < min_value:
                min_value = np.linalg.norm(diff)
        print(min_value)
        
        """
        print(self.map.loc)
        print(self.map.rot)
        
        print(self.map.grid)
        print(self.particles)
        print(self.weights)
        """

class MapWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title="Map Comparison")
        
def main():
    pfilter = ParticleFilter(100, 100, 1000, 100, 8)
    for x in range(20):
        pfilter.step()

if __name__ == "__main__":
    main()
