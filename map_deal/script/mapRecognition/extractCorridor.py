# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

WINDOW_NAME = "wallMap"



def debugText():
    print ('OpenCV Version: %s' % cv.__version__)


def extractCorridor(matIn, center):
    pass

def main():

    IMAGE_DIR = '/home/lzw/catkin_ws/src/map_deal/script/mapRecognition/pic/debug/floodfilled.jpg'

    debugText()

    # image = cv.imread("/home/liaoziwei/Desktop/mapRecognition/pic/humanmap_3L4th.png")
    image = cv.imread(IMAGE_DIR)
    im_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)


    dst = cv.inRange(im_gray, 30, 200)



    cv.imshow("dst", dst)
    cv.waitKey()

    cv.imwrite("./pic/debug/corridor.jpg", dst)


    # 腐蚀膨胀处理掉小碎点
    kernel = np.ones((5, 5), np.uint8)
    opening = cv.morphologyEx(dst, cv.MORPH_OPEN, kernel)
    cv.imshow("opening", opening)
    cv.waitKey()

    cv.imwrite("./pic/debug/corridor_dealt1.jpg", opening)

    # 腐蚀膨胀处理掉内部之前门留下的黑色小区域
    kernel = np.ones((5, 5), np.uint8)
    opening = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    cv.imshow("opening", opening)
    cv.waitKey()

    cv.imwrite("./pic/debug/corridor_dealt2.jpg", opening)

main()
