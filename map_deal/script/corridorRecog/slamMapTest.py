import cv2
import numpy

import cv2
import numpy

def dealSlamMap(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)

    # _, contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #
    # maxContour = []
    # maxSize = 0
    #
    # for i in range(len(contours)):
    #     if contours[i].size > maxSize:
    #         maxSize = contours[i].size
    #         maxContour = contours[i]
    #
    # #cv2.drawContours(img, [maxContour], -1, (0, 0, 255), 3)
    # rotrect = cv2.minAreaRect(maxContour)
    # box = cv2.boxPoints(rotrect)
    # box = numpy.int0(box)
    #
    # cv2.drawContours(img, [box], 0, (255, 0, 0), 3)

    cv2.namedWindow("SlamMap", cv2.WINDOW_NORMAL)
    cv2.imshow("SlamMap", binary)
    return binary

img = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/3L_Full.pgm')
img_dealt = dealSlamMap(img)



cv2.waitKey(0)
cv2.destroyAllWindows()
