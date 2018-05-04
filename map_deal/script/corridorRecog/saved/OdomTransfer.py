# Author : lzw
# function: get rid of wide zero area of the map; keep the transformation
# translated from c++ version, written for UImap Node under Ros

class odomTransfer:
    def __init__(self, imageMap, plainValue):
        self.leftBound = 0;
        self.rightBound = 0;
        self.upBound = 0;
        self.downBound = 0;

        self.fullMapRows = 0;
        self.fullMapCols = 0;

        self.map = imageMap.copy()
        self.plainValue = plainValue


    def change2MapCoordinate(self, searchPoints):
        searchPointsOnMap = []
        for i in range(len(searchPoints)):

            x = searchPoints[i][0] + self.leftBound;
            y = searchPoints[i][1] + self.upBound;

            realx = (x - self.fullMapCols/2.0)/self.fullMapCols *2.0 * 100.0;
            realy = -(y - self.fullMapRows/2.0)/self.fullMapRows *2.0 * 100.0;

            #ROS_INFO("realx:%f, realy:%f \n", realx, realy);

            temp = [realx, realy];
            searchPointsOnMap.append(temp);

        return searchPointsOnMap

    def getMapBound(self):
        mapin = self.map
        h, w = mapin.shape

        self.fullMapCols = h;
        self.fullMapRows = w;
        # **** need to be verified

        img = []
        img = mapin.copy();

        rows = self.fullMapRows
        cols = self.fullMapCols

        allIsBlank = True;

        self.leftBound = 0;
        self.upBound = 0;
        self.rightBound = cols;
        self.downBound = rows;

        center = [0, 0]
        x = 0
        y = 1
        center[x] = rows / 2;
        center[y] = cols / 2;

        for i in range(center[x], rows, 1):
            allIsBlank = True
            for j in range(0, cols, 1):
                value = img[j, i]
                if (value != self.plainValue):
                    allIsBlank = False
                    break

            if (allIsBlank):
                self.downBound = i
                break

        for i in range(center[x], 0, -1):
            allIsBlank = True
            for j in range(0, cols, 1):
                value = img[j, i]
                if (value != self.plainValue):
                    allIsBlank = False
                    break

            if (allIsBlank):
                self.upBound = i
                break

        for i in range(center[y], cols, 1):
            allIsBlank = True
            for j in range(0, rows, 1):
                value = img[i, j]
                if (value != self.plainValue):
                    allIsBlank = False
                    break

            if (allIsBlank):
                self.rightBound = i
                break

        for i in range(center[y], 0, -1):
            allIsBlank = True
            for j in range(0, rows, 1):
                value = img[i, j]
                if (value != self.plainValue):
                    allIsBlank = False
                    break

            if (allIsBlank):
                self.leftBound = i
                break

        print("left:%d up:%d right:%d down:%d \n" % (self.leftBound, self.upBound, self.rightBound, self.downBound))
        return

    def getMainMap(self , mapin, plainValue):
    # from center to find all is 205
        self.getMapBound()

        mapOut = mapin[self.leftBound:self.rightBound, self.upBound:self.downBound]
        return mapOut