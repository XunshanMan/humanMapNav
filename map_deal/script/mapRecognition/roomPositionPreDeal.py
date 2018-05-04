# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

WINDOW_NAME = "wallMap"


def deleteSmallPart(mapIn, minSize):
    # find all your connected components (white blobs in your image)
    nb_components, output, stats, centroids = cv.connectedComponentsWithStats(mapIn, connectivity=8)
    # connectedComponentswithStats yields every seperated component with information on each of them, such as size
    # the following part is just taking out the background which is also considered a component, but most of the time
    # we don't want that.
    sizes = stats[1:, -1]
    nb_components = nb_components - 1

    # minimum size of particles we want to keep (number of pixels)
    # here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
    min_size = minSize

    # your answer image
    img2 = np.zeros((output.shape))
    # for every component in the image, you keep it only if it's above min_size
    for i in range(0, nb_components):
        if sizes[i] >= min_size:
            img2[output == i + 1] = 255

    return img2


def BGRto2(mapIn, blockSize = 5 , C = 10):
    im_gray = cv.cvtColor(mapIn, cv.COLOR_BGR2GRAY)

    mapOut = cv.adaptiveThreshold(im_gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, blockSize, C)
    cv.bitwise_not(mapOut, mapOut)
    return mapOut


def to2(mapIn):
    im_gray = cv.cvtColor(mapIn, cv.COLOR_BGR2GRAY)
    _, mapOut = cv.threshold(im_gray, 50, 255, cv.THRESH_BINARY)
    cv.bitwise_not(mapOut, mapOut)
    return mapOut


def binary2SLAMmap(image):
    return

def debugText():
    print ('OpenCV Version: %s' % cv.__version__)



def main():
    THRESHOLD_WAY = 0   # 0 自适应阈值化  1 直接阈值化

    debugText()

    # image = cv.imread("/home/liaoziwei/Desktop/mapRecognition/pic/humanmap_3L4th.png")
    image = cv.imread("./pic/map.JPG")

    # 缩小图像，观察是否还会出现bug
    image = cv.resize(image, (image.shape[1]/3, image.shape[0]/3))

    cv.imshow("origin pic[resize]", image)
    cv.waitKey()

    if THRESHOLD_WAY == 0: # 自适应阈值化
        imageThreshold = BGRto2(image, 51, 10)
    else: # 直接阈值化
    # Solving Bug...
        imageThreshold = to2(image)
    # -----------
    #im_gray = cv.cvtColor(image, cv.COLOR_BGR2HSV)


    cv.imshow("imageThreshold", imageThreshold)
    cv.waitKey()
    cv.imwrite("./pic/debug/imageThreshold.jpg", imageThreshold)

    image2 = deleteSmallPart(imageThreshold, 800)
    cv.imshow(WINDOW_NAME, image2)
    cv.imwrite("./pic/debug/image2.jpg", image2)

    image_reverse = 255 - image2
    cv.imshow("bit not", image_reverse)
    cv.imwrite("./pic/debug/bitnot.jpg", image_reverse)

    cv.waitKey()


main()
