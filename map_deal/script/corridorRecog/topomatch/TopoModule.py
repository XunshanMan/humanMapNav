# -*- coding: utf-8 -*-


from numpy import *
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from time import time
from time import clock

PI = 3.14159

class mapMatcher(object):
    def __init__(self, slammap, humanmap, slampoint, humanpoint):
        print 'create a new matcher.'

        self.slammap = slammap.copy()
        self.humanmap = humanmap.copy()

        self.slam_h = slammap.shape[0]
        self.slam_w = slammap.shape[1]

        self.human_h = humanmap.shape[0]
        self.human_w = humanmap.shape[1]

        self.bigstart = []
        self.smallstart = []
        if(self.slam_h >= self.human_h and self.slam_w >= self.human_w):
            self.bigger = "slam_map"
            self.smaller = "human_map"
            self.bigmap = slammap.copy()
            self.smallmap = humanmap.copy()
            self.bigstart = slampoint
            self.smallstart = humanpoint
        elif (self.slam_h <= self.human_h and self.slam_w <= self.human_w):
            self.smaller = "slam_map"
            self.bigger = "human_map"
            self.smallmap = slammap.copy()
            self.bigmap = humanmap.copy()
            self.smallstart = slampoint
            self.bigstart = humanpoint
        else:
            print "error for map size."
            return

        self.smallmapOrigin = self.smallmap.copy()
        #cv2.imshow("smallmapOrigin", self.smallmapOrigin)
        self.bigmapOrigin = self.bigmap.copy()
        self.smallstartOrigin = self.smallstart

        self.rotate_angle = 0.0
        self.resize_scale = 0.0

        self.vector = [0, 0]
        self.updateVector()

        self.cost = 0.0
        # smallPoint + vector = BigPoint
        cv2.namedWindow("smallmap", cv2.WINDOW_NORMAL)
        cv2.imshow("smallmap", self.smallmap)

        self.bigmapCorridorPixels = count_nonzero(self.bigmap)

        # the switch for local_matching mode
        self.local_mode = True
        self.sideLength = 500
        self.local_center = tuple(self.small2big(self.smallstart))
        self.local_center_smallmap = self.smallstart

        # Use for recording matching results
        self.autoMatchResults = []

        print "Initialization succeeds:\n bigger map is %s(%d x %d); " \
              "Smaller map is %s(%d x %d), bigmapCorridorPixels:%d" % (self.bigger, self.bigmap.shape[0],self.bigmap.shape[1], self.smaller, self.smallmap.shape[0],self.smallmap.shape[1], self.bigmapCorridorPixels)

    def updateLocalCenter(self, centerOfSlamMapPixels):
        self.local_center = (centerOfSlamMapPixels[0], centerOfSlamMapPixels[1])
        self.local_center_smallmap = tuple(self.big2small(self.local_center))

    def updateVector(self):
        self.vector[0] = self.bigstart[0] - self.smallstart[0]
        self.vector[1] = self.bigstart[1] - self.smallstart[1]

    def big2small(self, point):
        self.updateVector()
        out = [0, 0]
        out[0] = point[0] - self.vector[0]
        out[1] = point[1] - self.vector[1]

        return out

    def small2big(self, point):
        self.updateVector()

        out = [0, 0]
        out[0] = point[0] + self.vector[0]
        out[1] = point[1] + self.vector[1]

        return out

    def rotateDelta(self, delta):
        self.rotate_angle += delta
        self.transferMap()

    def scaleDelta(self, delta):
        self.resize_scale += delta
        self.transferMap()

    def setRotate(self, rotate):
        self.rotate_angle = rotate
        self.transferMap()

    def setScale(self, scale):
        self.resize_scale = scale
        self.transferMap()

    def transferMap(self):
        rotateMatrix = cv2.getRotationMatrix2D(tuple(self.smallstart), self.rotate_angle, self.resize_scale)
        self.smallmap = cv2.warpAffine(self.smallmapOrigin, rotateMatrix, (self.smallmapOrigin.shape[1], self.smallmapOrigin.shape[0]))
        #self.updateSmallStart()

    def updateSmallStart(self):
        m = cv2.getRotationMatrix2D(tuple(self.smallstart), self.rotate_angle, self.resize_scale)
        x = self.smallstartOrigin[0]
        y = self.smallstartOrigin[1]

        self.smallstart[0] = x * m[0][0] + y * m[0][1] + m[0][2]
        self.smallstart[1] = x * m[1][0] + y * m[1][1] + m[1][2]

    def directTransfer(self, angle, scale):
        self.rotate_angle = angle
        self.resize_scale = scale
        self.transferMap()

    def compareCost(self):
        # based on the small map to calculate the best situation
        im = self.smallmap
        delta = zeros(im.shape, dtype=uint8)
        total_diff = 0
        total_same = 0
        corridorPixels = 0

        # Think about how to change into local cost.
        if self.local_mode:
            ymin = max(self.local_center_smallmap[1] - self.sideLength, 0)+ self.vector[1]
            ymax = min(self.local_center_smallmap[1] + self.sideLength, im.shape[0])+ self.vector[1]
            xmin = max(self.local_center_smallmap[0] - self.sideLength, 0)+ self.vector[0]
            xmax = min(self.local_center_smallmap[0] + self.sideLength, im.shape[1])+ self.vector[0]

            ########### why here is always 0??
            bigMapCorridor = count_nonzero(self.bigmap[ymin:ymax, xmin:xmax])
            #totalbigMapCorridor = count_nonzero(self.bigmap[:, :])
            #bigMapCorridor = count_nonzero(self.bigmap[xmin:xmax, ymin:ymax])

            for i in range(max(self.local_center_smallmap[1]-self.sideLength, 0), min(self.local_center_smallmap[1]+self.sideLength, im.shape[0]), 5):
                for j in range(max(self.local_center_smallmap[0]-self.sideLength, 0), min(self.local_center_smallmap[0]+self.sideLength, im.shape[1]), 5):
                    if im[i, j] < 250 :
                        continue
                    corridorPixels += 1
                    p = i + self.vector[1]
                    q = j + self.vector[0]
                    if(im[i][j] != self.bigmap[p][q]):
                        delta[i][j] = 255
                        total_diff += 1
                    else:
                        delta[i][j] = 100
                        total_same += 1
        else:
            for i in range(im.shape[0]):
                for j in range(im.shape[1]):
                    if im[i, j] < 250 :
                        continue
                    corridorPixels += 1
                    p = i + self.vector[1]
                    q = j + + self.vector[0]
                    if(im[i][j] != self.bigmap[p][q]):
                        delta[i][j] = 255
                        total_diff += 1
                    else:
                        delta[i][j] = 100
                        total_same += 1
            bigMapCorridor = self.bigmapCorridorPixelsp

        I = total_same
        U = corridorPixels + bigMapCorridor + 1
        #cost = total_diff / float(corridorPixels)
        cost = -I / float(U)

        cv2.imshow("mapDiff", delta)
        #cv2.namedWindow("bigmap", cv2.WINDOW_NORMAL)
        #cv2.imshow("bigmap", self.bigmap)
        #print " -----> Cost is: %d/%d(%d+%d) = %f totaldiff:%d" % (I, U ,corridorPixels, bigMapCorridor, cost, total_diff)
        self.cost = cost
        return cost

    def showRelation(self, mapname = "debug", local = False):
        # src = self.bigmapOrigin.copy()
        # img = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
        # im = self.smallmap

        # if local:
        #     for i in range(max(self.local_center_smallmap[1]-self.sideLength, 0), min(self.local_center_smallmap[1]+self.sideLength, im.shape[0]), 5):
        #         p = i + self.vector[1]
        #         for j in range(max(self.local_center_smallmap[0]-self.sideLength, 0), min(self.local_center_smallmap[0]+self.sideLength, im.shape[1]), 5):
        #             q = j + + self.vector[0]
        #             if (im[i][j] > 250):
        #                 # img[p][q][1] = 255
        #                 img[p][q][2] = 180
        #                 img[p][q][1] = 180
        #
        #             else:
        #                 # img[p][q][0] = 100
        #                 img[p][q][1] = 50
        # else:
        #     for i in range(im.shape[0]):
        #         p = i + self.vector[1]
        #         for j in range(im.shape[1]):
        #             q = j + + self.vector[0]
        #             if(im[i][j] > 250):
        #                 #img[p][q][1] = 255
        #                 img[p][q][2] = 180
        #                 img[p][q][1] = 180
        #
        #             else:
        #                 #img[p][q][0] = 100
        #                 img[p][q][1] = 50
        #                 #img[p][q][2] = 100

        # Faster add two pictures.

        im = self.smallmap.copy()
        src = self.bigmapOrigin.copy()

        maxLiney = min(self.vector[1]+im.shape[0], src.shape[0])
        maxLinex = min(self.vector[0]+im.shape[1], src.shape[1])

        img = zeros(tuple(list(src.shape)+[3]) , dtype=uint8)

        img[:,:,0] = 255-src[:, :]
        img[self.vector[1]: maxLiney, self.vector[0]:maxLinex, 2] = im[:, :]

        # draw the tilt center
        cv2.circle(img, tuple(self.bigstart), 13, (255, 0, 0), 5)
        cv2.circle(img, tuple(self.bigstart), 3, (255, 0, 0), 5)

        pointT = self.small2big(self.smallstart)
        cv2.circle(img, tuple(pointT), 13, (0, 255, 0), 5)
        cv2.circle(img, tuple(pointT), 3, (0, 255, 0), 5)

        if self.local_mode:
            rectp1 = (max(self.local_center[0]-self.sideLength, 0), max(self.local_center[1]-self.sideLength, 0))
            rectp2 = (min(self.local_center[0]+self.sideLength, im.shape[1]+self.vector[0]), min(self.local_center[1]+self.sideLength, im.shape[0]+self.vector[1]))

            cv2.rectangle(img, rectp1, rectp2, (0, 255, 255), thickness=5)
            cv2.circle(img, self.local_center, 20, (0, 255, 255), 5)

        cv2.imshow("relationship", img)

        #cv2.namedWindow("smallmap", cv2.WINDOW_NORMAL)
        cv2.imshow("smallmap", self.smallmap)
        cv2.imwrite("/home/liaoziwei/Desktop/mapMatchResult/" + mapname + ".jpg", img)

    def setbigstart(self,x, y):
        self.bigstart[0] = x
        self.bigstart[1] = y
        self.updateVector()

    def autoMatch_old(self, alpha, epsilon, max_itor):

        dtRlist = []
        dtSlist = []
        dtCostlist = []
        costlist = []

        dtR = 1
        dtS = 0.1

        totalStep = 0
        for i in range(max_itor):
            starttime = time()

            totalStep += 1
            dtRlist.append(dtR)
            dtSlist.append(dtS)

            (dtR, dtS, dtCost) = self.autoMatchStep(dtR, dtS, alpha)
            dtCostlist.append(dtCost)
            costlist.append(self.cost)


            stoptime = time()
            print "Step %d - dtR %f dtS %f dtCost %f cost %f, time: %ds" % (i, dtR, dtS, dtCost, self.cost, stoptime-starttime)
            if abs(dtCost) < epsilon:
                print 'dtCost(%d) is lower than epsilon(%d)' % (dtCost, epsilon)
                break

        x = range(totalStep)
        plt.figure(121)
        plt.plot(x, dtRlist, "g-", label="delta Rotate")
        plt.plot(x, dtSlist, "b-", label="delta Scale")
        plt.figure(122)

        plt.plot(x, dtCostlist, "g-", label="delta Cost")
        plt.plot(x, costlist, "b-", label="Cost")
        plt.title("Matching Result")

        plt.grid(True)
        plt.legend()
        plt.show()


    # this function updates rotation and scale saparately
    def autoMatchStep_oldversion(self, deltaRotate, deltaScale, alpha):
        self.rotateDelta(deltaRotate)

        costBegin = self.cost

        costNow = self.cost
        dcost = self.compareCost() - costNow
        # cost will be updated in function compareCost()
        nextRotate = -dcost * self.rotate_angle * alpha

        print "For Rotation: dcost %f nextRotate %f" % (dcost, nextRotate)

        self.scaleDelta(deltaScale)
        costNow = self.cost
        dcost = self.compareCost() - costNow
        # cost will be updated in function compareCost()
        nextScale = -dcost * self.resize_scale * alpha

        print "For Scale: dcost %f nextScale %f" % (dcost, nextScale)

        costDelta = self.cost - costBegin
        return nextRotate, nextScale, costDelta

    def autoMatch(self, alpha, max_itor):

        # update the record
        self.autoMatchResults = []

        direction = 1
        costLists = []
        loopscale = 1

        initialScale = self.resize_scale
        initialRotate = self.rotate_angle
        initialCost = self.cost

        positiveBestCost = 0.0
        positiveScale = 0.0
        positiveRotate = 0.0

        negativeBestCost = 0.0
        negativeScale = 0.0
        negativeRotate = 0.0

        costLists.append(self.cost)
        lastRotate = self.rotate_angle
        lastScale = self.resize_scale

        decreaseBeginCost = self.cost

        for i in range(max_itor):
            start = clock()
            self.scaleDelta(0.1 * direction * loopscale)
            print ("---")
            print ("begin step %d: lps %.1f cost %f rotate %f scale %f" % (i, loopscale, self.cost, self.rotate_angle, self.resize_scale))
            #angle, minCost = self.optimaizeRotate(3*loopscale, 1*loopscale)
            angle, minCost = self.optimaizeRotate(5*loopscale, 7)
            if minCost == 0:
                return 0, 0, 0, False
                # the position has some problem, choose last time.

            end = clock()
            print("[%f]step %d, direct %d, lps %.1f : scale %f, angle %f, cost %f") % \
                 ((end-start), i, direction, loopscale, self.resize_scale, self.rotate_angle, self.cost)
            self.showRelation(local=True)
            cv2.waitKey(30)

            if minCost > costLists[len(costLists) - 1]:
                if direction == 1:
                    direction = -1
                    positiveBestCost = costLists[len(costLists) - 1]
                    positiveRotate = lastRotate
                    positiveScale = lastScale
                    self.setScale(initialScale)
                    self.setRotate(initialRotate)
                    self.cost = decreaseBeginCost
                    print("Scale bigger to the end, so try smaller. scale: %f angle:%f" % (positiveScale, positiveRotate))


                else:
                    # -1 has also been end
                    negativeBestCost = costLists[len(costLists) - 1]
                    negativeRotate = lastRotate
                    negativeScale = lastScale
                    print("Scale smaller to the end, compare bigger and smaller... scale: %f angle:%f" % (negativeScale, negativeRotate))

                    if negativeBestCost > positiveBestCost:
                        #positive is better
                        print ("bigger is better.")
                        self.setRotate(positiveRotate)
                        self.setScale(positiveScale)
                        self.cost = positiveBestCost
                        decreaseBeginCost = positiveBestCost

                    else:
                        print ("smaller is better.")
                        self.setRotate(negativeRotate)
                        self.setScale(negativeScale)
                        self.cost = negativeBestCost
                        decreaseBeginCost = negativeBestCost

                    if loopscale > 0.5:
                        loopscale /= 10.0
                        direction = 1
                        print ("low down scale to %f, continue." % loopscale)
                        initialScale = self.resize_scale
                        initialRotate = self.rotate_angle

                    else:
                        print "All the plans have finished."
                        optimizeCost = self.cost-initialCost
                        if initialCost!=0:
                            print "Result cost from %f to %f delta: %f(%.1f%%)." % (initialCost, self.cost, optimizeCost,optimizeCost/float(initialCost)*100)
                        break
            costLists.append(self.cost)
            lastRotate = self.rotate_angle
            lastScale = self.resize_scale

        print "Match ends, final cost %f, rotate %f, scale %f" % \
              (self.cost, self.rotate_angle, self.resize_scale)

        return self.resize_scale, self.rotate_angle, self.vector, True

    def optimaizeRotate(self, maxrange, time = 7):
        nowRotate = self.rotate_angle
        costBegin = self.cost
        costLists=[]
        angleLists=[]
        for i in linspace(-maxrange, maxrange, time):
        #for i in [0]:

            self.setRotate(nowRotate + i)
            dcost = self.compareCost() - costBegin
            costLists.append(dcost)
            angleLists.append(nowRotate + i)
            #print "--> Rotate %.2f + %.2f : dt cost %f" % (nowRotate, i, dcost)

            # record matching data.
            self.autoMatchResults.append((self.resize_scale, self.rotate_angle, self.cost))

        mindtCost = min(costLists)
        bestR=angleLists[costLists.index(mindtCost)]
        self.setRotate(bestR)
        self.cost = costBegin + mindtCost
        return bestR, self.cost

    def setRobotPosition(self, x, y):
        self.local_center = (x, y)
        self.local_center_smallmap = (x-self.vector[0], y-self.vector[1])

    def updateSlamMap(self, newmap):
        self.bigmapOrigin = newmap.copy()
        self.slammap = newmap.copy()

        self.slam_h = newmap.shape[0]
        self.slam_w = newmap.shape[1]

        if(self.slam_h >= self.human_h and self.slam_w >= self.human_w):
            self.bigmap = newmap.copy()
        elif (self.slam_h <= self.human_h and self.slam_w <= self.human_w):
            self.smallmap = newmap.copy()
        else:
            print "error for map size."
            return

    def drawAutoMatchResult(self):
        if len(self.autoMatchResults) > 0:
            ax = plt.subplot(111, projection='3d')  # 创建一个三维的绘图工程

            for p in self.autoMatchResults:
                ax.scatter(p[0], p[1], p[2], c='y')

            ax.set_zlabel('cost')  # 坐标轴
            ax.set_ylabel('angle')
            ax.set_xlabel('scale')
            plt.show()




# The final useful function
def fromLineGetSearchPoints_web(pic, distance, number):
    gray = pic.copy()
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    graywall = cv2.inRange(gray, 128-10, 128+10)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    closed = cv2.morphologyEx(graywall, cv2.MORPH_CLOSE, kernel)
    #edges = cv2.Canny(wall, 50, 150, apertureSize=3)

    h, w = img.shape[:-1]

    cv2.namedWindow("imageLine", cv2.WINDOW_NORMAL)
    cv2.imshow("imageLine", closed)

    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/error_wall.jpg", graywall)
    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/error_img.jpg", img)
    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/error_closed.jpg", closed)

    cv2.waitKey(30)
    minLineLength = 40
    maxLineGap = 25
    lines = cv2.HoughLinesP(closed, 1, pi / 180, 50, minLineLength= minLineLength, maxLineGap=maxLineGap)

    if lines is None:
        print("There is No lines in the pic.\n")
        # come back, wait and try again
        return None, None, False

    print("Total lines: %d" % len(lines))
    if len(lines) < 2:
        # come back, wait and try agian
        return None, None, False



    wall = []

    theta = 0
    for i, line in enumerate(lines):
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            dx = x1 - x2
            dy = y1 - y2
            theta = math.atan2(dy, dx)
            x0 = -y1 / tan(theta) + x1
            if i is 0:
                wall.append([x0, theta])
                print("set line 1: (%f, %f)" % (x0, theta))

            else:
                wall0_theta = wall[0][1]
                wall0_x0 = wall[0][0]

                if (abs(theta - wall0_theta) < pi/180. * 5):

                    if(abs(x0 - wall0_x0) > maxLineGap):
                        wall.append([x0, theta])
                        print("find line 2: (%f, %f)" % (x0, theta))
                        break
            print x1,y1,x2,y2

        if len(wall) is 2:
            break

    if len(wall) < 2:
        print("Error in getting Wall")
        #    wall,img,closed
        #raise NameError("ErrorInGettingWall")
        # come back and try again.
        return None, None, False



    center = int((wall[0][0] + wall[1][0]) / 2)


    # Make Sure For the angle Direction.
    if abs(theta-pi/2.) < pi/4. or abs(theta) > 3* pi/4.:
        theta_fixed = theta - pi
    else:
        theta_fixed = theta

    if abs(theta_fixed+pi/2.) < pi/90.:
        point2_forDraw = (center, h)
    else:
        point2_forDraw = (h, int((h-center)*tan(theta_fixed)))

        if point2_forDraw[1] < 0:
            point2_forDraw = (0, int((0-center)*tan(theta_fixed)))

    cv2.line(img, (center, 0), (point2_forDraw[0], point2_forDraw[1]), (0, 0, 255), 2)
    # draw line



    picCenter = (h/2, w/2)

    searchPoints = []
    # image is a square, so h = w
    if abs(theta - pi/2.) < pi/4. or abs(theta + pi/2.) < pi/4.:
        # Down
        y_begin = int(picCenter[0])
        x_begin = int((y_begin - 0) / tan(theta) + center)

    else:
    #elif abs(theta - 0) < pi/4. or abs(theta - pi) < pi/4.:
        # left and right
        x_begin = int(picCenter[0])
        y_begin = int((x_begin - center) * tan(theta) + 0)

    searchPoints.append((x_begin, y_begin))
    for i in range(number - 1):
        # in the picture, y is oposite.!
        delta_theta = theta_fixed + pi
        y = y_begin + (i + 1) * distance * sin(delta_theta)
        x = x_begin + (i + 1) * distance * cos(delta_theta)
        searchPoints.append((x, y))


    for i in range(len(searchPoints)):

        cv2.circle(img, (int(searchPoints[i][0]), int(searchPoints[i][1])), 3, (255, 0, 0), 5)

    #cv2.imwrite('houghlines5.jpg', img)
    cv2.namedWindow("Result", cv2.WINDOW_NORMAL)

    cv2.imshow('Result', img)
    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/success_wall.jpg", graywall)
    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/success_img.jpg", img)
    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/success_closed.jpg", closed)

    cv2.waitKey(30)
    # circle of the function  -pi~pi
    if theta>3*pi/4:
        theta_fixed = theta - pi
    elif theta < -3*pi/4:
        theta_fixed = theta + pi
    elif theta > pi/4 and theta < 3*pi/4:
        theta_fixed = theta - pi
    else:
        theta_fixed = theta

    return searchPoints, theta_fixed, True

# ---- no use
def fromLineGetSearchPoints(pic):
    mapLine = cv2.inRange(pic, 128-10, 128+10)

    cv2.namedWindow("imageLine", cv2.WINDOW_NORMAL)
    cv2.imshow("imageLine", mapLine)
    cv2.waitKey()

    lines = cv2.HoughLinesP(mapLine, 1, pi / 180, 100)

    result = cv2.cvtColor(pic, cv2.COLOR_GRAY2BGR)

    for line in lines[0]:
        rho = line[0]
        theta = line[0]
        print rho
        print theta
        if (theta < (pi / 4.)) or (theta > (3. * pi / 4.0)):

            pt1 = (int(rho / cos(theta)), 0)

            pt2 = (int((rho - result.shape[0] * sin(theta)) / cos(theta)), result.shape[0])

            cv2.line(result, pt1, pt2, (0, 255, 255), 3)
        else:

            pt1 = (0, int(rho / sin(theta)))

            pt2 = (result.shape[1], int((rho - result.shape[1] * cos(theta)) / sin(theta)))

            cv2.line(result, pt1, pt2, (0, 255, 255), 3)
    cv2.namedWindow("Result", cv2.WINDOW_NORMAL)

    cv2.imshow('Result', result)
    cv2.imwrite("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/imageROI_result.jpg" , result)

    #cv2.waitKey(0)
    #cv2.destroyAllWindows()