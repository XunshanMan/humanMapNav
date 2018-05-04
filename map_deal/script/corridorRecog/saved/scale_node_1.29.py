# this program is for testing and debugging
# the main function can be used to test the automation adjustment on multiple files.

# 1.29
# Bug: After Robot gets Final Position. Points seem to be not sent.

# 1.12
# add a new function:
# when the robot move a distance, it will send a message.

import cv2
import numpy
import time as tm

from ClassModule import *

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32MultiArray
from tf2_msgs.msg import TFMessage

# using tf frame to directly transform from map->base_link
from tfTransModule import *

_map = OccupancyGrid()
_empty = True
newCommand = False
match_vector = []
match_rotate = 0.0

homestate = [0, 0, 0.0]

PosesLastTime = [0.0 , 0.0]

#for recording odoms
amclPoses = []
#amclPixels = []   # pixels on big slam map
amclGetNewData = False
imageForShow = []
map2odom = []
amclPixels_UnderMap = []

# Record all the destinations
destinations = []
new_destination = False

new_map = False
mapMain = []  # Type: Gray


#DEBUG = True
DEBUG = False

# whether let the robot stop, when it comes into the destination circle
DESTINATION_CIRCLE = 80

from OdomTransfer import *

mt = None

# calculate the total times of matching
match_times = 0

# to judge the robot's state
flag_sentSearchGoal = False

# Old function.
# After directly using tf to get the robot position, it's useless.

def tf_callback(data):
    # first, not consider about map ->  odom.
    #rospy.loginfo(rospy.get_caller_id() + " [%d]%s -> %s" % (len(data.transforms), data.transforms[0].header.frame_id, data.transforms[0].child_frame_id))
    if data.transforms[0].child_frame_id == 'base_link' and data.transforms[0].header.frame_id == 'odom':
        global amclPoses
        # for odom -> base_link
        x = data.transforms[0].transform.translation.x
        y = data.transforms[0].transform.translation.y
        qz = data.transforms[0].transform.rotation.z
        qw = data.transforms[0].transform.rotation.w
        point = [x, y]
        amclPoses.append(point+[qz, qw])
        global amclGetNewData
        amclGetNewData = True
        #print("get amclPoses data: total %d"%len(amclPoses))
    elif data.transforms[0].header.frame_id == 'map' and data.transforms[0].child_frame_id == 'odom':
        global map2odom
        x = data.transforms[0].transform.translation.x
        y = data.transforms[0].transform.translation.y
        qz = data.transforms[0].transform.rotation.z
        qw = data.transforms[0].transform.rotation.w
        point = [x, y]
        map2odom.append(point+[qz, qw])
        #print("get map2odom data: total %d"%len(map2odom))

        # wait for dealing amclPoses, so no need to get a flag here

def drawOdom(imageForShow, pixel, angle, color=1):
    if color == 1:
        color_circle = (255, 0, 0)
        color_line = (125, 125, 125)
    elif color == 2:
        color_circle = (0, 255, 255)
        color_line = (125, 125, 125)
    elif color == 3 :
        color_circle = (0, 0, 255)
        color_line = (125, 125, 125)



    #amclPixels.append(pixel + [angle])
    cv2.circle(imageForShow, (pixel[0], pixel[1]), 3, color_circle)

    lineLength = 40
    pt2 = [0, 0]
    pt2[0] = int(pixel[0] + lineLength * math.cos(angle))
    pt2[1] = int(pixel[1] + lineLength * math.sin(angle))

    cv2.line(imageForShow, tuple(pixel), tuple(pt2), color_line)

def dealOdomData(oT, tf_listener):
    global amclGetNewData
    global imageForShow
    #global amclPoses

    ######## We don't need to subscribe tf topic, so the flag for getting data from tf
    # is useless

#if amclGetNewData:

    # # ------------- Origin Just ODOM
    # pixels = oT.change2MapPixels([amclPoses[len(amclPoses) - 1]])
    # pixel = pixels[0]
    #
    # qz = amclPoses[len(amclPoses) - 1][2]
    # qw = amclPoses[len(amclPoses) - 1][3]
    #
    # angle = -oT.quaternion2angle(qz, qw) - math.pi/2
    # # when +90, the direction is clockwise
    # # - is for the opposite in the picture and in the map
    #
    # # y pixel is opposite in map and picture.
    #
    # drawOdom(imageForShow, pixel, angle)
    # #cv2.imshow("StateNow", imageForShow)
    # # ------------------------------
    #
    # #---------- this part is for adjusting map relationship.
    # global amclPixels_UnderMap
    #
    # global map2odom
    # if len(map2odom) > 0:
    #     m2onow = map2odom[len(map2odom) - 1]
    #     map2odom_angle = oT.quaternion2angle(m2onow[2], m2onow[3])
    # else:
    #     m2onow = [0,0]
    #     map2odom_angle = 0
    # odomIn = [amclPoses[len(amclPoses) - 1][0] + m2onow[0], amclPoses[len(amclPoses) - 1][1] + m2onow[1]]
    # pixels_m2o = oT.change2MapPixels([odomIn])
    # pixel_m2o = pixels_m2o[0]
    # angle_m2o = angle - map2odom_angle
    #
    # #amclPixels_UnderMap.append(pixel_m2o + [angle_m2o])
    #
    # drawOdom(imageForShow, pixel_m2o, angle_m2o, color=2)

    # ----- tf function
    base_link = PoseStamped()
    # base_link.pose.position.x = amclPoses[len(amclPoses) - 1][0]
    # base_link.pose.position.y = amclPoses[len(amclPoses) - 1][1]
    # base_link.pose.position.z = 0
    #
    # base_link.pose.orientation.z = amclPoses[len(amclPoses) - 1][2]
    # base_link.pose.orientation.w = amclPoses[len(amclPoses) - 1][3]
    mapPoint = PoseStamped()
    mapPoint = transformPoint_new(tf_listener, base_link)



    amclGetNewData = False

    # ---- as the same as part 1
    pose_tf = [mapPoint.pose.position.x,
               mapPoint.pose.position.y,
               mapPoint.pose.orientation.z,
               mapPoint.pose.orientation.w]

    # When it can't get the tf relationship, it returns zero. Let's delete this kind of situation
    if(abs(mapPoint.pose.position.x) < 0.1 and abs(mapPoint.pose.position.y) < 0.1):
        print ("[TF]can not get the new data.")
        return


    print ("[TF]Succeed in getting the new data.")

    pixels = oT.change2MapPixels([pose_tf])
    pixel = pixels[0]

    qz = mapPoint.pose.orientation.z
    qw = mapPoint.pose.orientation.w

    angle = -oT.quaternion2angle(qz, qw) - math.pi/2

    global amclPixels_UnderMap
    amclPixels_UnderMap.append(pixel + [angle])
    # Only place for changing amclPixels_UnderMap

    drawOdom(imageForShow, pixel, angle ,color=3)




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

    #cv2.namedWindow("img" + n, cv2.WINDOW_NORMAL)
    #cv2.imshow("img" + n, img)
    #cv2.imshow("img" + n, mask)

    return mask

def dealSlamMap(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)

    #cv2.namedWindow("SlamMap", cv2.WINDOW_NORMAL)
    #cv2.imshow("SlamMap", binary)
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
    if(event == cv2.EVENT_RBUTTONUP):
        print "Click."
        param[0].setRobotPosition(x, y)
        scale, match_rotate, match_vector = AutoMatching(param[1])
        print ( "[AutoMatch]Final: scale: %f, rotate: %f, vector %d,%d" % (scale, match_rotate,match_vector[0],match_vector[1] ))
        param[0].showRelation()

        # param[0].setbigstart(x, y)
        # param[0].showRelation()
        # param[0].compareCost()


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

    global mt
    if mt == None:
        mt = mapMatcher(imgSlam_dealt, imgHuman_dealt, start_slam, start_human)
        if mt== None:
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
    #cv2.setMouseCallback("relationship", onMouse, mt)

    state = 0
    state_auto = 1
    match_times = 0
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
            match_times = match_times + 1
            print "AutoMatching begin, total times: %d" % match_times

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

    # for 3L
    #imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
    #imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))

    imgHuman = cv2.imread('/home/liaoziwei/Desktop/mapRecognition/pic/map_3G_SLAMmap.jpg')
    imgHuman_dealt = deal(imgHuman, "- Human", (410, 179))


    # templatemath_img(imgSlam_dealt, imgHuman_dealt, 0.5)
    start_slam = [2000, 2000]  # initial point on slam map
    #start_human = [132, 778]
    start_human = [homestate[0], homestate[1]]

    initialRotate = int(-(homestate[2]- math.pi/2.0)/math.pi*180.0)

    global mt
    if mt == None:
        mt = mapMatcher(imgSlam_dealt, imgHuman_dealt, start_slam, start_human)
        if mt== None:
            return
    else:
        #update the map
        mt.updateSlamMap(imgSlam_dealt)

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
    cv2.setMouseCallback("relationship", onMouse, [mt, slamMap])

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

    if DEBUG is True:
        print "**********FOR DEBUGGING********"
        scale = 1.59
        rotate = 0
        mt.updateVector()
        vector = mt.vector
    else:
        scale, rotate, vector, resultflag = mt.autoMatch(3, 50)
        if resultflag is False:
            return scale, rotate, vector, False
    global match_times
    match_times += 1
    mt.showRelation("Match"+ match_times.__str__() + "_" + mt.resize_scale.__str__())

    state_auto = auto

    return scale, rotate, vector, True

# --------- for ros node -----------
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " get map.")
    global _map
    global new_map
    new_map = True
    _map = data



def getCommand(data):
    rospy.loginfo(rospy.get_caller_id() + " get command.")
    global newCommand
    newCommand = True

    homestate[0] = int(data.data[0]) #x
    homestate[1] = int(data.data[1]) #y
    homestate[2] = data.data[2] #angle

# Deal with the Final Point to make sure it is in the map. Before Send the command to
# dn_finder node.
def getSearchCommand(data):
    rospy.loginfo(rospy.get_caller_id() + " get destination. *******")

    global destinations
    global new_destination
    new_destination = True

    dest = data.data
    destinations.append(dest)
#
#     commandPoint.data.push_back(mapPoints[firstID].x);
#     commandPoint.data.push_back(mapPoints[firstID].y);
#     commandPoint.data.push_back(homePoint.x);
#     commandPoint.data.push_back(homePoint.y);
#     commandPoint.data.push_back(homeAngle);
#     commandPoint.data.push_back(DoorNumber);



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
    rospy.Subscriber("/command_getscale", Float32MultiArray, getCommand)
    #rospy.Subscriber("/tf", TFMessage, tf_callback)

    # To get the Final Destination. And Print IT On The Picture!
    rospy.Subscriber("/searchCommand", Float32MultiArray, getSearchCommand)


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
            value = imageMap[i][index]
            if value == -1:
                imageMap[i][index] = 255
            elif value == 100:
                imageMap[i][index] = 128
            else:
                imageMap[i][index] = 0

        #print index


    print("succeed in analyzing slam map.")
    # print set
    imageMap = numpy.ndarray.astype(imageMap, numpy.uint8)
    imageMap = 255 - imageMap  # fast speed

    # Initial dealing for the same picture.
    imageMap = cv2.rotate(imageMap, cv2.ROTATE_180)


    #cv2.namedWindow("testMap", cv2.WINDOW_NORMAL)
    #cv2.imshow("testMap", imageMap)
    #cv2.waitKey()

    return imageMap, oT

# DIS is the distance of (x, y) / meter.
def analyzeDistance(pub, DIS):
    global amclPixels_UnderMap

    if len(amclPixels_UnderMap) < 1:
        return
    global PosesLastTime
    newPose = amclPixels_UnderMap[len(amclPixels_UnderMap) - 1]
    distance = sqrt((PosesLastTime[0] - newPose[0]) ** 2 + (PosesLastTime[1] - newPose[1]) ** 2)
    if distance > DIS:
        pub.publish(True)
        PosesLastTime = newPose
        rospy.loginfo("The Robot Has MOVED %d meter" % DIS)

def calculateAndSendSearchingGoal(oT):

    # calculate and send goal.


    # get the picture around robot's position and final goal.

    rospy.loginfo("Begin calculating the final searching points.")

    robotpos = amclPixels_UnderMap[len(amclPixels_UnderMap) - 1]
    x = robotpos[0]
    y = robotpos[1]
    global mapMain
    imageROI = mapMain[ y-DESTINATION_CIRCLE*3:y+DESTINATION_CIRCLE*3, x-DESTINATION_CIRCLE*3 : x+DESTINATION_CIRCLE*3]

    cv2.namedWindow("imageROI", cv2.WINDOW_NORMAL)
    cv2.imshow("imageROI", imageROI)

    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/imageROI.jpg" , imageROI)

    points, theta = fromLineGetSearchPoints_web(imageROI, 25, 5)
    # detect the line!
    # choose the center point!
    points_onImageMap = []
    for i in range(len(points)):
        px = points[i][0] + x - DESTINATION_CIRCLE*3
        py = points[i][1] + y - DESTINATION_CIRCLE*3
        points_onImageMap.append((px, py))

    points_coordinate = oT.change2MapCoordinate_fullMap(points_onImageMap)

    # temply Generating used to have empty publish content. Try to generate at the beginning.
    points_puber = rospy.Publisher("/searchCommand_points", Float32MultiArray, queue_size=1)

    # write the communication with UImap.... 1.28.
    # check the logic, to make sure the function can end successfully
    # UIMAP FLAG:     // CONTINUE ON COMMUNICATION

    pubData = Float32MultiArray()
    pubData.data.append(theta)
    for i in range(len(points_coordinate)):
        pubData.data.append(points_coordinate[i][0])
        pubData.data.append(points_coordinate[i][1])

    points_puber.publish(pubData)

    rospy.loginfo("Send Final Searching Points finished.")




def analyzeDistanceToDestination(pub, DIS, oT):
    global amclPixels_UnderMap
    global destinations
    if DIS > 0:
        if len(destinations) < 1:
            return
        if len(amclPixels_UnderMap) < 1 :
            return
        newPose = amclPixels_UnderMap[len(amclPixels_UnderMap) - 1]
        newDest = destinations[len(destinations) - 1]
        pose_tf = [newDest[0], newDest[1]]
        pixels = oT.change2MapPixels([pose_tf])
        pixel = pixels[0]

        x = int(pixel[0])
        y = int(pixel[1])

        distance = sqrt((x - newPose[0]) ** 2 + (y - newPose[1]) ** 2)
        if distance < DIS:
            pub.publish(False)
            rospy.loginfo("The Robot Has GOTTEN TO THE DESTINATION CIRCLE. DIS:%.1f" % DIS)

            # Now Calculate the Searching Goal.
            calculateAndSendSearchingGoal(oT)

            global flag_sentSearchGoal
            flag_sentSearchGoal = True
            return

def dealWithNewMap():
    global new_map
    if new_map == True:
        global imageForShow
        global mapMain
        mapMain, oT = analyzeRosMap()
        print("analyze ros map finished.")
        imageForShow = mapMain.copy()
        imageForShow = cv2.cvtColor(imageForShow, cv2.COLOR_GRAY2BGR)

        # ---------- Draw all the Destinations
        # global new_destination
        # if new_destination is True:
        #     print("Dealing new destination data.")
        #     new_destination = False
        global destinations
        for j, i in enumerate(destinations):
            x0 = int(i[0])
            y0 = int(i[1])
            pose_tf = [x0,y0]
            pixels = oT.change2MapPixels([pose_tf])
            pixel = pixels[0]

            x = int(pixel[0])
            y = int(pixel[1])
            cv2.circle(imageForShow, (x, y), 6, (0, 0, 255), 2)
            cv2.circle(imageForShow, (x, y), 10, (0, 0, 255), 3)
            cv2.circle(imageForShow, (x, y), 20, (255, 0, 0), 4)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(imageForShow, "%d" % j, (x+5, y+5), font, 2, (0, 255, 255), 2, cv2.LINE_AA)

            if j== (len(destinations)-1):
                # Draw the Destination Area for the newest destination
                global DESTINATION_CIRCLE
                rCircle = DESTINATION_CIRCLE
                cv2.circle(imageForShow, (x, y), rCircle, (0, 255, 255), 3)


        # ---------- Draw all the Odoms.
        for j, i in enumerate(amclPixels_UnderMap):
            color_circle = (0, 0, 255)
            color_line = (125, 125, 125)

            cv2.circle(imageForShow, (i[0], i[1]), 3, color_circle)

            lineLength = 40
            pt2 = [0, 0]
            pt2[0] = int(i[0] + lineLength * math.cos(i[2]))
            pt2[1] = int(i[1] + lineLength * math.sin(i[2]))

            cv2.line(imageForShow, (i[0], i[1]), tuple(pt2), color_line)



        new_map = False


def main_node():
    node_initial()
    global _map
    global newCommand

    # for 3L human map
    #imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')

    imgHuman = cv2.imread('/home/liaoziwei/Desktop/mapRecognition/pic/map_3G_SLAMmap.jpg')

    #cv2.imwrite( "/home/liaoziwei/Desktop/runSpace/map.jpg", imgHuman)
    #cv2.imshow("map", imgHuman)
    #cv2.waitKey(30)

    # for 3L human map
    # imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))
    imgHuman_dealt = deal(imgHuman, "- Human", (410, 179))

    cv2.imwrite( "/home/liaoziwei/Desktop/runSpace/map_dealt.jpg", imgHuman_dealt)


    puber = rospy.Publisher("/map_scale", Float32MultiArray, queue_size=1)
    puber_distanceChange = rospy.Publisher("/map_distance_change", Bool, queue_size=1)
    tf_listener = TransformListener(rospy.Duration(10))

    rate = rospy.Rate(1)  # 10hz
    sleep5 = rospy.Rate(0.2)
    rateHz5 = rospy.Rate(5)

    time_last = 0

    oTExists_flag = False
    global imageForShow
    global match_vector
    global mt
    print "wait for command. oTExist: %d" % oTExists_flag
    pubDataSaved = []

    global DESTINATION_CIRCLE

    global flag_sentSearchGoal
    while True:
#        print "wait for command. oTExist: %d" % oTExists_flag
        if newCommand == False:
            rateHz5.sleep()
            analyzeDistance(puber_distanceChange, 15)
            if oTExists_flag:
                dealOdomData(oT, tf_listener)

                if flag_sentSearchGoal is False:
                    analyzeDistanceToDestination(puber_distanceChange, DESTINATION_CIRCLE, oT)
                    if flag_sentSearchGoal is True:
                        while(rospy.is_shutdown() is False):
                            print("Wait For Work Done.")
                            cv2.waitKey(30)
                else:
                    # Wait For Detecting Door Number?
                    # Useless, calculate by other Program.

                    # The algorithm ends here.
                    return

                dealWithNewMap()
                if len(amclPixels_UnderMap) > 0:
                    mt.updateLocalCenter(amclPixels_UnderMap[len(amclPixels_UnderMap)-1])
                cv2.namedWindow("imageForShow", cv2.WINDOW_NORMAL)
                cv2.imshow("imageForShow", imageForShow)
                global match_times
                cv2.imwrite("/home/liaoziwei/Desktop/mapMatchResult/Goal_" + match_times.__str__() + ".jpg", imageForShow)

                cv2.waitKey(30)
            continue
        print "get command."
        if (tm.time() - time_last) < 5:
            print "Two times need 5 seconds."
            newCommand = False
            continue
        while True:
            h = _map.info.height
            if h == 0:
                rate.sleep()
                print "wait for map"
                continue
            else:
                break

        dealWithNewMap()

        print("begin analyzing the scale.")
        mapMain, oT = analyzeRosMap()
        print("analyze ros map finished.")
        imageForShow = mapMain.copy()
        imageForShow = cv2.cvtColor(imageForShow, cv2.COLOR_GRAY2BGR)
        oTExists_flag = True

        scale, match_rotate, match_vector, resultflag = AutoMatching(mapMain)

        # add the transition of odom->map



        if resultflag is True:
            pubData = Float32MultiArray()
            pubData.data.append(scale)
            pubData.data.append(match_vector[0])
            pubData.data.append(match_vector[1])
            pubData.data.append(match_rotate/180.0*math.pi)   # rotate changes to pi
            print ("Publish scale: %f, rotate: %f, vector %d,%d" % (
            scale, match_rotate, match_vector[0], match_vector[1]))

        else:
            if len(pubDataSaved) > 0:
                pubData = pubDataSaved[len(pubDataSaved)-1]
            else:

                print ("Position ERROR, Choose last time.")
                newCommand = False
                time_last = tm.time()

        puber.publish(pubData)
        pubDataSaved.append(pubData)
        newCommand = False
        time_last = tm.time()

if __name__=='__main__':
    #main()
    #every time receive a request, then publish the result.

    main_node()

    rospy.spin()