#!/usr/bin/env python

from topomapClass import *
from OdomTransfer import *

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16MultiArray

# for amcl Message
from geometry_msgs.msg import PoseWithCovarianceStamped
# for tf message
from tf2_msgs.msg import TFMessage


map = None
slamMap = OccupancyGrid()

imageForShow = []
amclPoses = []
amclPixels = []
# --- dealing with SLAM Map ----

# for odom transfer
map_upBound = 0
map_leftBound = 0

amclGetNewData = False

def map_callback(data):
    global slamMap
    slamMap = data

def analyzeRosMap():
    # keep the contours.
    # ATTENTION: Y Axis will be different from rviz, because y is down for the positive

    global slamMap

    if slamMap.info.height is 0:
        return 0, 0, 0

    rospy.loginfo(rospy.get_caller_id() + " get map.")
    map_height = slamMap.info.height
    map_width = slamMap.info.width
    imageMap = np.asarray(slamMap.data).reshape((map_height, map_width))
    #imageMap = imageMap.T   # change back fthe order of X, Y ; I don't know why reshape will change them.
    oT = odomTransfer(imageMap, -1)
    oT.getMapBound()

    global map_upBound
    global map_leftBound
    map_upBound = oT.upBound
    map_leftBound = oT.leftBound

    for index in range(oT.upBound, oT.downBound, 1):
        for i in range(oT.leftBound, oT.rightBound, 1):
            if imageMap[i][index] == -1:
                imageMap[i][index] = 128
            elif imageMap[i][index] == 0:
                imageMap[i][index] = 0
            elif imageMap[i][index] == 100:
                imageMap[i][index] = 255
        #print index

    # # print set
    imageMap = np.ndarray.astype(imageMap, np.uint8)
    imageMap = 255 - imageMap  # fast speed
    #
    ## Initial dealing for the same picture.
    # imageMap = cv2.rotate(imageMap, cv2.ROTATE_180)
    imageMap = cv2.flip(imageMap, 0)

    return [imageMap, 1, oT]




def createMap():
    map = topoMap("3L")
    map.initialNode()

    map.addNode(nodeDirection.left, "CROSS")
    map.addNode(nodeDirection.down, "3L401")
    map.addNode(nodeDirection.down, "3L402")
    map.addNode(nodeDirection.down, "3L403")

    map.drawNodeMap()

    return map

def listener():
    print 'begin listening...'
    rospy.init_node('topomap_node')

    rospy.Subscriber("/topo_update_position", Int16MultiArray, callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " get data.")
    map.setNewPosition(data.data[0])

def amcl_callback(data, oT):
    rospy.loginfo(rospy.get_caller_id() + " get amcl_pose data.")
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    point = [x, y]
    amclPoses.append(point)
    pixels = oT.change2MapPixels([point])
    angle = oT.quaternion2angle(qz, qw)
    amclPixels.append(pixels[0]+[angle])
    global amclGetNewData
    amclGetNewData = True
    return

def tf_callback_noOt(data):
    # first, not consider about map ->  odom.
    rospy.loginfo(rospy.get_caller_id() + " [%d]%s -> %s" % (len(data.transforms), data.transforms[0].header.frame_id, data.transforms[0].child_frame_id))

# copy from module OdomTransfer
def change2MapPixels(searchPoints):
    searchPointsOnPixels = []

    global map_leftBound
    global map_upBound
    for i in range(len(searchPoints)):
        x = searchPoints[i][0] / 200.0 * 4000 + 4000/2.0
        y = -searchPoints[i][1] / 200.0 * 4000 + 4000/2.0

        #ROS_INFO("realx:%f, realy:%f \n", realx, realy);
        realx =  int(x - map_leftBound);
        realy =  int(y - map_upBound);

        temp = [realx, realy];
        searchPointsOnPixels.append(temp);

    return searchPointsOnPixels

def quaternion2angle(qz, qw):
    return math.atan2(2*qw*qz, 1-2*qz*qz)


def tf_callback(data):
    # first, not consider about map ->  odom.
    #rospy.loginfo(rospy.get_caller_id() + " [%d]%s -> %s" % (len(data.transforms), data.transforms[0].header.frame_id, data.transforms[0].child_frame_id))
    if data.transforms[0].child_frame_id != 'base_link':
        return
    x = data.transforms[0].transform.translation.x
    y = data.transforms[0].transform.translation.y
    qz = data.transforms[0].transform.rotation.z
    qw = data.transforms[0].transform.rotation.w
    point = [x, y]
    amclPoses.append(point+[qz, qw])
    global amclGetNewData
    amclGetNewData = True


def drawOdoms(point, angle):
    cv2.circle(imageForShow, tuple(point), 5, (0, 0, 255))

    lineLength = 10
    pt2 = [0, 0]
    pt2[0] = point[0] + lineLength * math.cos(angle)
    pt2[1] = point[1] + lineLength * math.sin(angle)

    cv2.line(imageForShow, tuple(point), tuple(pt2), (0, 0, 255))
    cv2.imshow("StateNow", imageForShow)

def main():
    global map
    global slamMap
    map = createMap()

    listener()

    while 1:
        (slamMapImage, result, oT) = analyzeRosMap()

        if result is 1:
            break
        print("wait for slamMap.")


    puber = rospy.Publisher("/topo_next_goal", Int16MultiArray, queue_size=1)
    #get tf data.
    #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback, oT)
    rospy.Subscriber("/tf", TFMessage, tf_callback)

    #rospy.Subscriber("/tf", TFMessage, tf_callback, oT, queue_size=10)

    # oT = odomTransfer(slamMap, -1)
    # oT.getMapBound()
    # slamMapMain = oT.getMainMap(slamMapImage)

    cv2.namedWindow("slamMapImage", cv2.WINDOW_NORMAL)
    cv2.imshow("slamMapImage", slamMapImage)
    #cv2.waitKey()

    global imageForShow
    imageForShow = cv2.cvtColor(slamMapImage, cv2.COLOR_GRAY2BGR)

    cv2.namedWindow("StateNow", cv2.WINDOW_NORMAL)
    cv2.imshow("StateNow", imageForShow)

    global amclGetNewData
    while True:
        cv2.waitKey(300)
        if amclGetNewData:
            pixels = oT.change2MapPixels([amclPoses[len(amclPoses)-1]])
            pixel = pixels[0]

            qz = amclPoses[len(amclPoses) - 1][2]
            qw = amclPoses[len(amclPoses) - 1][3]

            angle = -oT.quaternion2angle(qz, qw)
            # - is for the opposite in the picture and in the map

            # y pixel is opposite in map and picture.

            amclPixels.append(pixel + [angle])
            cv2.circle(imageForShow, (pixel[0], pixel[1]), 3, (255, 0, 0))

            lineLength = 40
            pt2 = [0, 0]
            pt2[0] = int(pixel[0] + lineLength * math.cos(angle))
            pt2[1] = int(pixel[1] + lineLength * math.sin(angle))

            cv2.line(imageForShow, tuple(pixel), tuple(pt2), (0, 0, 255))
            cv2.imshow("StateNow", imageForShow)

            amclGetNewData = False

    rospy.spin()



if __name__=='__main__':
    main()