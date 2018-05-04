import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

WINDOW_NAME = "canny_resize"


def getRGBtoCanny(imageIn):
	image_gray = cv.cvtColor(imageIn, cv.COLOR_BGR2GRAY)

	paramLow = cv.getTrackbarPos("param1", WINDOW_NAME)
	paramHigh = cv.getTrackbarPos("param2", WINDOW_NAME)

	image_canny = cv.Canny(image_gray, paramLow, paramHigh)

	return image_canny

def doCanny(a):
	image_resize_canny = getRGBtoCanny(image_resize)
	cv.imshow(WINDOW_NAME, image_resize_canny)

def main():
	image = cv.imread("/home/liaoziwei/Desktop/mapRecognition/pic/map_3G.jpg")

	height, width = image.shape[:2]

	global image_resize
	image_resize = cv.resize(image, (width/5, height/5), interpolation=cv.INTER_CUBIC)

	cv.imshow(WINDOW_NAME, image_resize)

	cv.waitKey()

main()
