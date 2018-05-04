from numpy import *
import cv2
import matplotlib.pyplot as plt
from time import time
from time import clock


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

        print "Initialization succeeds:\n bigger map is %s(%d x %d); " \
              "Smaller map is %s(%d x %d)" % (self.bigger, self.bigmap.shape[0],self.bigmap.shape[1], self.smaller, self.smallmap.shape[0],self.smallmap.shape[1])

    def updateVector(self):
        self.vector[0] = self.bigstart[0] - self.smallstart[0]
        self.vector[1] = self.bigstart[1] - self.smallstart[1]

    def big2small(self, point):
        self.updateVector()
        return point - self.vector

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
        delta = zeros(im.shape)
        total_diff = 0
        total_same = 0
        corridorPixels = 0
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

        #cost = total_diff / float(corridorPixels)
        cost = -total_same

        cv2.imshow("mapDiff", delta)
        #print "total difference: %d/%d = %f" % (total, corridorPixels , cost)
        self.cost = cost
        return cost

    def showRelation(self, mapname = "debug"):
        src = self.bigmapOrigin.copy()
        img = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
        im = self.smallmap
        for i in range(im.shape[0]):
            p = i + self.vector[1]
            for j in range(im.shape[1]):
                q = j + + self.vector[0]
                if(im[i][j] > 250):
                    #img[p][q][1] = 255
                    img[p][q][2] = 180
                    img[p][q][1] = 180

                else:
                    #img[p][q][0] = 100
                    img[p][q][1] = 50
                    #img[p][q][2] = 100

        cv2.circle(img, tuple(self.bigstart), 13, (255, 0, 0), 5)
        cv2.circle(img, tuple(self.bigstart), 3, (255, 0, 0), 5)

        pointT = self.small2big(self.smallstart)
        cv2.circle(img, tuple(pointT), 13, (0, 255, 0), 5)
        cv2.circle(img, tuple(pointT), 3, (0, 255, 0), 5)

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
            angle, minCost = self.optimaizeRotate(1*loopscale, 1)

            end = clock()
            print("[%f]step %d, direct %d, lps %.1f : scale %f, angle %f, cost %f") % \
                 ((end-start), i, direction, loopscale, self.resize_scale, self.rotate_angle, self.cost)
            self.showRelation()
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
                        print "Result cost from %f to %f delta: %f(%.1f%%)." % (initialCost, self.cost, optimizeCost,optimizeCost/float(initialCost)*100)
                        break
            costLists.append(self.cost)
            lastRotate = self.rotate_angle
            lastScale = self.resize_scale

        print "Match ends, final cost %f, rotate %f, scale %f" % \
              (self.cost, self.rotate_angle, self.resize_scale)

        return self.resize_scale

    def optimaizeRotate(self, maxrange, time = 7):
        nowRotate = self.rotate_angle
        costBegin = self.cost
        costLists=[]
        angleLists=[]
       # for i in linspace(-maxrange, maxrange, time):
        for i in [0]:

            self.setRotate(nowRotate + i)
            dcost = self.compareCost() - costBegin
            costLists.append(dcost)
            angleLists.append(nowRotate + i)
            print "--> Rotate %.2f + %.2f : dt cost %f" % (nowRotate, i, dcost)

        mindtCost = min(costLists)
        bestR=angleLists[costLists.index(mindtCost)]
        self.setRotate(bestR)
        self.cost = costBegin + mindtCost
        return bestR, self.cost

