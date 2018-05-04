from AmclProModule import *
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import LaserScan

import math
import random

laserdatas = []

DEBUG_MODE = False
wait_for_initial = False

MAIN_WINDOW_NAME = 'Map in AmclPro'
#SHOW_WINDOW_NAME = 'What\'s going on'
SHOW_WINDOW_NAME = MAIN_WINDOW_NAME


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " get data.")
    laserdatas.append(data)



def listener():
    print 'begin listening...'
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('amclPro')

    rospy.Subscriber("/scan", LaserScan, callback)

def select_initial(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(param[0].get_showimage(), (x,y), 3, (255,0,0), -1)
        global wait_for_initial
        wait_for_initial = False
        param[1].append((x,y))    # add the initial points
    elif event == cv2.EVENT_MBUTTONDOWN:
        #param[0].initialize((x,y), 1, 3, 100, 0.001)

        param[0].particle_create((x,y), -math.pi/2.0)

        cv2.imshow(SHOW_WINDOW_NAME, param[0].get_showimage())

def generateParticles(amcl, n):
    print 'begin to generate ', n, ' particles...'
    plt.figure()
    plt.subplot(n/3+1, 3, 1)

    for i in range(n):
        state = RobotState(random.randint(0, amcl.get_map_shape()[0]), random.randint(0, amcl.get_map_shape()[1]), random.randint(0, 360)/360.0*2*math.pi - math.pi)
        xs, ys = amcl.particle_view(state)
        amcl.draw_particle_view(state, xs, ys)

        plt.subplot(n/3+1, 3, i+1)
        plt.scatter(xs, ys, marker='.', color='blue')

        print 'finish the ', i, ' particle'

    plt.ion()


def testParticles():
    print 'Program begins...'

    maploc = '/home/liaoziwei/Desktop/superAmcl/map/humanmap_3L4th_SLAMmap.jpg'
    amcl = AmclPro(maploc)

    if amcl.show_state() == 0:
        return

    cv2.imshow('Map in AmclPro', amcl.get_map())
    cv2.waitKey()

    generateParticles(amcl, 20)
    cv2.imshow('show image', amcl.get_showimage())
    cv2.waitKey()

    while True:
        plt.pause(0.05)

    return


def main():
    print 'Program begins...'

    maploc = '/home/liaoziwei/Desktop/superAmcl/map/humanmap_3L4th_SLAMmap.jpg'
    amcl = AmclPro(maploc)
    amcl.set_rotate_angle(math.pi/2.0)

    if amcl.show_state() == 0:
        return

    cv2.namedWindow(MAIN_WINDOW_NAME)
    cv2.imshow(MAIN_WINDOW_NAME, amcl.get_map())

    initial_point = []
    cv2.setMouseCallback(MAIN_WINDOW_NAME, select_initial, [amcl, initial_point])

    global wait_for_initial
    wait_for_initial = True
    print 'Please select the initial point...'
    while wait_for_initial:
        if cv2.waitKey(30) == 'q':
            return
    print 'get the select point (%d, %d)' % (initial_point[0][0], initial_point[0][1])

    # startpoint, particles_num, average_meter, pixnum
    amcl.initialize(initial_point[0], 12, 1, 100)   # temply change to 0

    cv2.imshow(SHOW_WINDOW_NAME, amcl.get_showimage())
    cv2.waitKey()


    # begin receiving data and get test.


    listener()

    print 'Listen initialization finishes.'

    global laserdatas
    time = 0
    while True:
        dataTotal = len(laserdatas)
        cv2.waitKey(30)
        plt.pause(0.01)

        if dataTotal < 1:
            print 'wait for data...'
            continue
        if time > 5:
            newestdata = laserdatas[dataTotal-1]
            #amcl.locate(newestdata)
            amcl.plotScan(newestdata)
            time = 0
        time += 1

    rospy.spin()

    return

if __name__=='__main__':
    if not DEBUG_MODE:
        main()
    else:
        testParticles()
