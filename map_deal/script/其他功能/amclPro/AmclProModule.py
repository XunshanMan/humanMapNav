import cv2
import math
import numpy as np
import random

import matplotlib.pyplot as plt

from enum import Enum, unique
# a class for enum
@unique
class ParticleConsist(Enum):
    PARTICLE_STATE = 0
    PARTICLE_XS = 1
    PARTICLE_YS = 2


class Point(object):
    def __init__(self, xParam=0.0, yParam=0.0):
        self.x = xParam
        self.y = yParam

    def __str__(self):
        return "(%.2f, %.2f)" % (self.x, self.y)

    def distance(self, pt):
        xDiff = self.x - pt.x
        yDiff = self.y - pt.y
        return math.sqrt(xDiff ** 2 + yDiff ** 2)

    def sum(self, pt):
        newPt = Point()
        xNew = self.x + pt.x
        yNew = self.y + pt.y
        return Point(xNew, yNew)

class RobotState:
    def __init__(self, x = 0, y = 0, angle = 0.0):
        self.x = x
        self.y = y
        self.angle = angle

class AmclPro(object):
    def __init__(self, maplocation):
        self.maplocation = maplocation

        self.map = cv2.imread(self.maplocation)
        self.showmap = self.map.copy()
        self.mapys = self.map.shape[0]
        self.mapxs = self.map.shape[1]

        if self.map is None:
            print 'Error: can\'t read the map. Location: ', self.maplocation
            self.state = 0
        else:
            print 'Succeed in reading map.'
            self.state = 1

        self.VALUE_OCCUPIED = 0
        self.drawtimes = 0

        # param initialization
        self.startpoint = (0, 0)
        self.particles_num = 0

        self.scale = 25
        self.average_meter = 0
        self.particles = []
        self.particles_sight = []
        self.pixnum = 100

        self.particles_plot = []
        self.rotate_angle = 0.0

        plt.figure()
        plt.ion()
        plt.pause(0.01)

    def set_rotate_angle(self, angle):
        self.rotate_angle = angle

    def get_map(self):
        return self.map

    def get_map_shape(self):
        return self.mapxs, self.mapys


    def show_state(self):
        return self.state

    def locate(self, data):

        # now the point comes!
        # using gradient decent algorithm to guess the position in the human map.
        return

    def sendRidar(self, state, angle, deltpix = 1, maxDis = 100):
        value = 0.0
        xbase = state.x
        ybase = state.y

        flag_foundOccupied = False
        xbase += deltpix * math.cos(angle)
        ybase += deltpix * math.sin(angle)

        totalDis = 0.0
        while xbase < self.map.shape[1] and ybase < self.map.shape[0] and xbase>0 and ybase>0:
            value = self.map[int(ybase)][int(xbase)][0]
            if(value == self.VALUE_OCCUPIED):
                flag_foundOccupied=True
                break
            xbase += deltpix * math.cos(angle)
            ybase += deltpix * math.sin(angle)
            totalDis += deltpix

            if totalDis > maxDis:
                break

        if flag_foundOccupied:
            return xbase - state.x, ybase - state.y
        else:
            return None, None

    def direction_calibration(self, points_x, points_y):
        # rotate_angle = self.rotate_angle
        # point_rotatex = []
        # point_rotatey = []
        # for i in range(len(points_x)):
        #     point_rotatex.append(points_x[i] * math.cos(rotate_angle) - points_y[i] * math.sin(rotate_angle))
        #     point_rotatey.append(points_x[i] * math.sin(rotate_angle) + points_y[i] * math.cos(rotate_angle))
        newy = []
        newx = []
        for i in points_y:
            if(i is None):
                newy.append(None)
            else:
                newy.append(-i)
        for i in points_x:
            if(i is None):
                newx.append(None)
            else:
                newx.append(-i)
        return newy, newx

    def particle_see_single(self, state):
        # as the ridar has 725 data points. So we also get so many data points.
        angle_increment = 0.00613592332229
        angle_max = 2.09234976768
        angle_min = -2.35619449615

        points_x = []
        points_y = []

        for i in range(self.pixnum):
            angle = state.angle + angle_min + i*angle_increment * 725/self.pixnum
            if angle > math.pi:
                angle -= 2 * math.pi
            elif angle < -math.pi:
                angle += 2 * math.pi
            x, y = self.sendRidar(state, angle)
            points_x.append(x)
            points_y.append(y)

        return points_x, points_y

    def particle_draw_single(self, particle, number):
        state = particle[0]
        points_x = particle[1]
        points_y = particle[2]

        cv2.circle(self.showmap, (state.x, state.y), 4, (255, 0, 0))
        cv2.putText(self.showmap, number.__str__(), (state.x + 5, state.y + 5), 0, 0.5, (0, 0, 255), 2)

        x2 = state.x + 12 * math.cos(state.angle)
        y2 = state.y + 12 * math.sin(state.angle)
        cv2.line(self.showmap, (state.x, state.y), (int(x2), int(y2)), (255, 255, 0), thickness=1)

        for i in range(len(points_x)):
            if points_x[i] == None:
                continue

            # angle_2pi = state.angle
            # if angle_2pi < 0:
            #     angle_2pi += math.pi * 2
            # rotate_angle = - math.pi/2.0 + angle_2pi
            # rotate_angle = -rotate_angle
            #
            # rotate_angle = 0
            #
            # point_rotatex = points_x[i] * math.cos(rotate_angle) - points_y[i] * math.sin(rotate_angle)
            # point_rotatey = points_x[i] * math.sin(rotate_angle) + points_y[i] * math.cos(rotate_angle)

            cv2.circle(self.showmap, (int(points_x[i]+state.x), int(points_y[i]+state.y)), 2, (0, 0, 255))

    def get_showimage(self):
        return self.showmap

    def particle_generate(self, num, sigma):
        print 'begin to generate', num, 'particles...'

        rantrix = np.random.normal(loc=0.0, scale=sigma, size=(num, 2))
        rantrix_pixels = rantrix * self.scale * self.average_meter

        for i in range(num):
            robotx = rantrix_pixels[i][0] + self.startpoint[0]
            roboty = rantrix_pixels[i][1] + self.startpoint[1]
            robotangle = random.randint(0, 360) / 360.0 * 2 * math.pi - math.pi

            state = RobotState(int(robotx), int(roboty), robotangle)
            xs, ys = self.particle_see_single(state)
            self.particles.append([state, xs, ys])
            self.particles_plot.append([state, xs, ys])
            print 'succeed in the ', i+1, ' particle.'
        self.plot_refresh()
        self.particles_num = len(self.particles)
        print 'generation has been done.', ' Total:', self.particles_num

    def particle_see(self, particles):
        for i in range(len(particles)):
            sight = self.particle_see_single(particles[i][0])
            self.particles_sight.append(sight)

    def particle_draw(self):
        self.drawtimes += 1
        for i in range(len(self.particles)):
            self.particle_draw_single(self.particles[i], i+1)


    def plot_add_particle(self, particle):
        self.particles_plot.append(particle)
        print 'add succeeds. total showing plot:', len(self.particles_plot)
        self.plot_refresh()

    def plot_refresh(self):
        total = len(self.particles_plot)

        plt.subplot(total/3+2, 3, 4)

        for i in range(total):
            plt.subplot(total / 3 + 2, 3, i+1+3)
            rotatex, rotatey = self.direction_calibration(self.particles_plot[i][1], self.particles_plot[i][2])
            plt.scatter(rotatex, rotatey, marker='.', color='blue')

    def plotScan(self, data):
        plt.subplot(len(self.particles_plot)/3+2, 3, 1)
        plt.cla()
        points = data.ranges
        plt.scatter(0, 0, marker='.', color='red')

        xs = []
        ys = []
        for i in range(len(points)):
            if points[i] < 0.02:
                continue
            rad = data.angle_min + data.angle_increment * i
            x = points[i] * math.cos(rad)
            y = points[i] * math.sin(rad)
            xs.append(x)
            ys.append(y)

        plt.scatter(xs, ys, marker='.', color='blue')

    def particle_create(self, point, angle):
        state = RobotState(int(point[0]), int(point[1]), angle)
        xs, ys = self.particle_see_single(state)
        self.particles.append([state, xs, ys])

        self.particles_num = len(self.particles)

        newParticle = self.particles[self.particles_num-1]
        a = []
        a.append(newParticle)
        self.particle_see(a)
        self.particle_draw_single(newParticle, self.particles_num)
        print 'creation has been done.', ' Total:', self.particles_num
        self.plot_add_particle(newParticle)

    def initialize(self, startpoint, particles_num, average_meter, pixnum, sigma = 1.0):
        self.startpoint = startpoint
        self.average_meter = average_meter
        self.pixnum = pixnum

        self.particle_generate(particles_num, sigma)
        self.particle_see(self.particles)
        self.particle_draw()

# Particle's consist:
# [state, xs, ys]