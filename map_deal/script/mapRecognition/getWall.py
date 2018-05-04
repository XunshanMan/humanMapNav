import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

WINDOW_NAME = "wallMap"


def deleteSmallPart(mapIn, minSize):
	#find all your connected components (white blobs in your image)
	nb_components, output, stats, centroids = cv.connectedComponentsWithStats(mapIn, connectivity=8)
	#connectedComponentswithStats yields every seperated component with information on each of them, such as size
	#the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
	sizes = stats[1:, -1]; nb_components = nb_components - 1

	# minimum size of particles we want to keep (number of pixels)
	#here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
	min_size = minSize  

	#your answer image
	img2 = np.zeros((output.shape))
	#for every component in the image, you keep it only if it's above min_size
	for i in range(0, nb_components):
	    if sizes[i] >= min_size:
		img2[output == i + 1] = 255

	return img2

def BGRto2(mapIn):
	im_gray = cv.cvtColor(mapIn, cv.COLOR_BGR2GRAY)

	mapOut = cv.adaptiveThreshold(im_gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 5, 10)
	cv.bitwise_not(mapOut, mapOut)
	return mapOut

def binary2SLAMmap(image):
	return

def main():
	#image = cv.imread("/home/liaoziwei/Desktop/mapRecognition/pic/humanmap_3L4th.png")	
	image = cv.imread("/home/liaoziwei/Desktop/mapRecognition/pic/map_3G.jpg")
	cv.imshow("origin pic", image)
	cv.waitKey()
	
	imageThreshold = BGRto2(image);
	cv.imshow("imageThreshold", imageThreshold)
	cv.waitKey()
	cv.imwrite("/home/liaoziwei/Desktop/mapRecognition/pic/debug/imageThreshold.jpg",imageThreshold)

	image2 = deleteSmallPart(imageThreshold, 250)
	cv.imshow(WINDOW_NAME, image2)
	cv.imwrite("/home/liaoziwei/Desktop/mapRecognition/pic/debug/image2.jpg",image2)

	image_reverse = 255 - image2
	cv.imshow("bit not", image_reverse)
	cv.imwrite("/home/liaoziwei/Desktop/mapRecognition/pic/debug/bitnot.jpg",image_reverse)
	

	cv.waitKey()

main()
