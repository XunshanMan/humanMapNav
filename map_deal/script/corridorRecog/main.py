# coding: utf-8
# this program is for testing and debugging
# the main function can be used to test the automation adjustment on multiple files.

import cv2
import numpy

from ClassModule import *

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

        initialRotate = 90

    elif map_chosen == 2:
        # 此处增加对于新主楼的MAP测试
        #imgSlam = cv2.imread('/home/lzw/catkin_ws/src/map_deal/script/corridorRecog/gridmap/part_full_startTo.pgm')
        imgSlam = cv2.imread('/home/lzw/catkin_ws/src/map_deal/script/corridorRecog/gridmap/full_3A.pgm')

        imgSlam_dealt = dealSlamMap(imgSlam)

        #imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
        #imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))
        # deal函数将人类地图中的走廊提取为二值图，该二值图为inrange函数直接得到
        # 由于之前用另一套算法手动提取了新主楼人类地图的走廊，因此在此处直接读取提取的结果即可
        # 注意直接读取时要按灰度图读取
        # imgHuman_dealt = cv2.imread('/home/lzw/catkin_ws/src/map_deal/script/mapRecognition/pic/map_corridor.jpg', cv2.IMREAD_GRAYSCALE)

        # 经过扩充之后的地图
        imgHuman_dealt = cv2.imread('/home/lzw/catkin_ws/src/map_deal/script/mapRecognition/pic/'
                                    'map_corridor_expand.jpg', cv2.IMREAD_GRAYSCALE)


        # 显示两张地图，并且获得起始位置分别的坐标点

        # 先设定窗口可以自由缩放，不然图片过大
        cv2.namedWindow("slam map", cv2.WINDOW_NORMAL)
        cv2.namedWindow("human map", cv2.WINDOW_NORMAL)

        cv2.imshow("slam map", imgSlam_dealt)
        cv2.imshow("human map", imgHuman_dealt)
        cv2.waitKey()

        #templatemath_img(imgSlam_dealt, imgHuman_dealt, 0.5)
        start_slam = [2000, 2000]
        # 实际上为建图时的起始中心

        # 原人类地图走廊参数
        # start_human = [786, 446]
        # 扩展人类地图走廊参数
        start_human = [1123, 941]

        initialRotate = 270

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
    state_auto = 0
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
            mt.drawAutoMatchResult()
            mt.showRelation(maps[id] + "_" + mt.resize_scale.__str__())

            state_auto = auto

        if cv2.waitKey(30) == "q":
            break
    cv2.destroyAllWindows()

def listAllGo(id):
    map_chosen = 2
    maps = ["start.pgm", "start2.pgm", "start3.pgm", "start4_wc.pgm", "start5_wc.pgm",
            "start6.pgm", "start7_stairs.pgm", "start8.pgm", "start9_end.pgm"]

    if map_chosen == 1:
        imgSlam = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/3L_Full.pgm')
        imgSlam_dealt = dealSlamMap(imgSlam)

        imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
        imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))

        # templatemath_img(imgSlam_dealt, imgHuman_dealt, 0.5)
        start_slam = [2214, 2603]
        start_human = [132, 778]
    else:
        imgSlam = cv2.imread('/home/liaoziwei/ros_project/map/map_matching/' + maps[id])
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
    scale = cv2.getTrackbarPos('scale', "mapDiff")
    rotate = cv2.getTrackbarPos('rotate', "mapDiff")
    s = cv2.getTrackbarPos(switch, "mapDiff")
    auto = cv2.getTrackbarPos(switch_auto, "mapDiff")

    print "auto get on"

    mt.directTransfer(rotate, scale / 100.0)
    mt.showRelation()
    mt.compareCost()
    mt.autoMatch(3, 100)
    mt.showRelation(maps[id] + "_" + mt.resize_scale.__str__())

    state_auto = auto


    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
    #total = 9  #9
    #for i in range(total):
    #    listAllGo(i)
    #    print "finish the number ", i