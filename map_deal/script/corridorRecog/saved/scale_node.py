# this program is for testing and debugging
# the main function can be used to test the automation adjustment on multiple files.


import cv2
import numpy

from ClassModule import *

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32

_map = OccupancyGrid()
_empty = True
newCommand = False

from OdomTransfer import *

def deal(img, n, seed_pt):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    maxContour = []
    maxSize = 0

    for i in range(len(contours)):
        if contours[i].size > maxSize:
            maxSize = contours[i].size
            maxContour = contours[i]

    #cv2.drawContours(img, [maxContour], -1, (0, 0, 255), 3)
    rotrect = cv2.minAreaRect(maxContour)
    box = cv2.boxPoints(rotrect)
    box = numpy.int0(box)

    cv2.drawContours(img, [box], 0, (0, 0, 0), 3)

    #def floodFill(image, mask, seedPoint, newVal, loDiff=None, upDiff=None, flags=None): # real signature unknown; restored from __doc__
    h, w = img.shape[:2]
    mask = numpy.zeros((h+2, w+2), numpy.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # closed = cv2.morphologyEx(img2, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("Close", closed);
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    #cv2.imshow("Open", opened);

    cv2.floodFill(img, mask, seed_pt, (125, 125, 125),
                  (10,) * 3, (10,) * 3, 4)
    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/map_highlight.jpg", img)

    mask = cv2.inRange(img, (124,124,124), (126,126,126))

    cv2.namedWindow("img" + n, cv2.WINDOW_NORMAL)
    #cv2.imshow("img" + n, img)
    cv2.imshow("img" + n, mask)

    return mask

def dealSlamMap(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)

    cv2.namedWindow("SlamMap", cv2.WINDOW_NORMAL)
    cv2.imshow("SlamMap", binary)
    return binary

def templatemath_img(image,Target,value):

    imageSrc = image
    template = Target
    w, h = template.shape[:]
    res = cv2.matchTemplate(imageSrc,template,cv2.TM_CCOEFF_NORMED)
    threshold = value
    loc = numpy.where( res >= threshold)

    imgOut = cv2.cvtColor(imageSrc, cv2.COLOR_GRAY2BGR)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(imgOut, pt, (pt[0] + w, pt[1] + h), (7,249,151), 2)

    cv2.namedWindow("Detected", cv2.WINDOW_NORMAL)
    cv2.imshow('Detected',imgOut)

    cv2.namedWindow("DetectedRES", cv2.WINDOW_NORMAL)
    cv2.imshow('DetectedRES',res)

def match(slammap,humanmap,slampoint,humanpoint):

    #mt.compareCost()
    cv2.waitKey(0)

def onMouse(event, x, y, flags, param):
    if(event == cv2.EVENT_LBUTTONUP):
        print "Click."
        param.setbigstart(x, y)
        param.showRelation()
        param.compareCost()


def nothing(argument):
    pass

def main():

    map_chosen = 2
    maps = ["start.pgm", "start2.pgm", "start3.pgm", "start4_wc.pgm", "start5_wc.pgm",
            "start6.pgm", "start7_stairs.pgm", "start8.pgm", "start9_end.pgm", ]
    id = 1

    if map_chosen == 1:
        imgSlam = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/3L_Full.pgm')
        imgSlam_dealt = dealSlamMap(imgSlam)

        imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
        imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))

        #templatemath_img(imgSlam_dealt, imgHuman_dealt, 0.5)
        start_slam = [2214, 2603]
        start_human = [132, 778]
    else:
        imgSlam = cv2.imread('/home/liaoziwei/ros_project/map/map_matching/' + maps[id] )
        imgSlam_dealt = dealSlamMap(imgSlam)

        imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
        imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))

        # templatemath_img(imgSlam_dealt, imgHuman_dealt, 0.5)
        start_slam = [2005, 2001]
        start_human = [132, 778]

        initialRotate = 270

    mt = mapMatcher(imgSlam_dealt, imgHuman_dealt, start_slam, start_human)
    if mt == None:
        return

    cv2.namedWindow("mapDiff", cv2.WINDOW_NORMAL)
    cv2.createTrackbar('scale', "mapDiff", 100, 200, nothing)
    cv2.createTrackbar('rotate', "mapDiff", initialRotate, 360, nothing)
   # cv2.createTrackbar('scale', "mapDiff", 154, 200, nothing)
   # cv2.createTrackbar('rotate', "mapDiff", 15, 360, nothing)

    switch = '0 : OFF \n1 : ON'
    cv2.createTrackbar(switch, "mapDiff", 0, 1, nothing)

    # automation adjustment
    switch_auto = '0 : AutoA \n1 : AutoB'
    cv2.createTrackbar(switch_auto, "mapDiff", 0, 1, nothing)

    cv2.namedWindow("relationship", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("relationship", onMouse, mt)

    state = 0
    state_auto = 1
    while(1):
        scale = cv2.getTrackbarPos('scale', "mapDiff")
        rotate = cv2.getTrackbarPos('rotate', "mapDiff")
        s = cv2.getTrackbarPos(switch, "mapDiff")
        auto = cv2.getTrackbarPos(switch_auto, "mapDiff")

        if(s != state):
            print "get on"

            mt.directTransfer(rotate, scale/100.0)
            mt.showRelation()
            mt.compareCost()
            state = s
        if(auto != state_auto):
            print "auto get on"

            mt.directTransfer(rotate, scale/100.0)
            mt.showRelation()
            mt.compareCost()
            mt.autoMatch(3, 100)
            mt.showRelation(maps[id] + "_" + mt.resize_scale.__str__())

            state_auto = auto

        if cv2.waitKey(30) == "q":
            break
    cv2.destroyAllWindows()

def AutoMatching(slamMap):
    imgSlam_dealt = slamMap.copy()

    imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
    imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))

    # templatemath_img(imgSlam_dealt, imgHuman_dealt, 0.5)
    start_slam = [2005, 2001]
    start_human = [132, 778]

    initialRotate = 0

    mt = mapMatcher(imgSlam_dealt, imgHuman_dealt, start_slam, start_human)
    if mt == None:
        return

    cv2.namedWindow("mapDiff", cv2.WINDOW_NORMAL)
    cv2.createTrackbar('scale', "mapDiff", 100, 200, nothing)
    cv2.createTrackbar('rotate', "mapDiff", initialRotate, 360, nothing)
    # cv2.createTrackbar('scale', "mapDiff", 154, 200, nothing)
    # cv2.createTrackbar('rotate', "mapDiff", 15, 360, nothing)

    switch = '0 : OFF \n1 : ON'
    cv2.createTrackbar(switch, "mapDiff", 0, 1, nothing)

    # automation adjustment
    switch_auto = '0 : AutoA \n1 : AutoB'
    cv2.createTrackbar(switch_auto, "mapDiff", 0, 1, nothing)

    cv2.namedWindow("relationship", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("relationship", onMouse, mt)

    state = 0
    state_auto = 1
    scale = cv2.getTrackbarPos('scale', "mapDiff")
    rotate = cv2.getTrackbarPos('rotate', "mapDiff")
    s = cv2.getTrackbarPos(switch, "mapDiff")
    auto = cv2.getTrackbarPos(switch_auto, "mapDiff")

    print "auto get on"

    mt.directTransfer(rotate, scale / 100.0)
    mt.showRelation()
    mt.compareCost()
    scale = mt.autoMatch(3, 100)
    mt.showRelation("rosNode" + "_" + mt.resize_scale.__str__())

    state_auto = auto

    cv2.destroyAllWindows()

    return scale

# --------- for ros node -----------
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " get map.")
    global _map
    _map = data

def getCommand(data):
    rospy.loginfo(rospy.get_caller_id() + " get command.")
    global newCommand
    if(data.data == True):
        newCommand = True


def listener():
    print 'begin listening...'
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('scale_node')

    rospy.Subscriber("/map", OccupancyGrid, callback)
    # listenning for commands
    rospy.Subscriber("/command_getscale", Bool, getCommand)

def node_initial():
    print("initializing the node")
    listener()

def analyzeRosMap():
    global _map
    map_height = _map.info.height
    map_width = _map.info.width
    imageMap = numpy.asarray(_map.data).reshape((map_height, map_width))
    imageMap = imageMap.T   # change back fthe order of X, Y ; I don't know why reshape will change them.
    # to resize map and get a smaller one
    # BIGGEST PROBLEM: When comparing, we have not considered about the possibility of
    # map's size. It's too complicated.
    oT = odomTransfer(imageMap, -1)
    oT.getMapBound()
    # map_main = oT.getMainMap(imageMap, -1)
    #
    #
    # cv2.imshow("testMap", map_main)
    # cv2.waitKey()
    # for index, value in enumerate(imageMap):
    #     for i, v in enumerate(value):
    #         if v == -1:
    #             value[i] = 0
    #         elif v < 50:
    #             value[i] = 128
    #         elif v > 50:
    #             value[i] = 255
    #     print index
    for index in range(oT.upBound, oT.downBound, 1):
        for i in range(oT.leftBound, oT.rightBound, 1):
            if imageMap[i][index] == -1:
                imageMap[i][index] = 255
            else:
                imageMap[i][index] = 0
        print index

    # print set
    imageMap = numpy.ndarray.astype(imageMap, numpy.uint8)
    imageMap = 255 - imageMap  # fast speed

    # Initial dealing for the same picture.
    imageMap = cv2.rotate(imageMap, cv2.ROTATE_180)


    cv2.namedWindow("testMap", cv2.WINDOW_NORMAL)
    cv2.imshow("testMap", imageMap)
    #cv2.waitKey()

    return imageMap


if __name__=='__main__':
    #main()
    #every time receive a request, then publish the result.

    node_initial()
    global _map
    global newCommand

    imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
    cv2.imwrite( "/home/liaoziwei/Desktop/runSpace/map.jpg", imgHuman)
    cv2.imshow("map", imgHuman)
    cv2.waitKey()

    imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))


    puber = rospy.Publisher("/map_scale", Float32, queue_size=1)
    rate = rospy.Rate(10)  # 10hz
    sleep5 = rospy.Rate(0.2)
    while True:

        if newCommand == False:
            print "wait for command."
            rate.sleep()
            continue

        while True:
            h = _map.info.height
            if h == 0:
                rate.sleep()
                print "wait for map"
                continue
            else:
                break

        mapMain = analyzeRosMap()
        scale = AutoMatching(mapMain)

        puber.publish(scale)
        print ( "Publish scale: %f" % scale  )
        sleep5.sleep()
        newCommand = False

    rospy.spin()