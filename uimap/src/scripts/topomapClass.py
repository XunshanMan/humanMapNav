from enum import Enum, unique

import cv2
import numpy as np

@unique
class nodeType(Enum):
    unknown = 0
    corridor = 1
    cross = 2

@unique
class nodeDirection(Enum):
    up = 0
    down = 1
    left = 2
    right = 3


class topoNode(object):
    def __init__(self, ID, type, up, down, left, right, leftRoom, rightRoom, name):
        self.ID = ID
        self.name = name
        self.type = type
        self.left = left
        self.up = up
        self.down = down
        self.right = right

        self.leftRoom = leftRoom
        self.rightRoom = rightRoom

        self.lastNode = None

        self.goalcost = 0





class topoMap(object):
    def __init__(self, mapname):
        self.mapname = mapname
        self.nodeLists = []

        self.nodeNum = 0

        self.nodesDrawn = set()

        # for position
        self.position = 0
        self.nextdirection = 0
        self.nextgoal = 0
        print("create a new topologyMap: %s" % self.mapname)

    def createNode(self, type, name = "", leftRoom = "", rightRoom = ""):
        if name == "":
            name = "Node" + str(self.nodeNum)
        node = topoNode(self.nodeNum, type, None, None, None, None, leftRoom, rightRoom, name=name)
        self.nodeLists.append(node)
        self.nodeNum += 1

        self.lastNode = self.nodeLists[self.nodeNum-1]

        print("a new node is generated. total: %d" % self.nodeNum)
        return self.nodeLists[self.nodeNum-1]

    def initialNode(self):
        # the first time to run
        return self.createNode(nodeType.unknown)

    def addNode(self, direction, name, leftroom="", rightroom=""):
        lstNode = self.lastNode
        node = self.createNode(nodeType.corridor, name, leftroom, rightroom)
        self.combineNode(lstNode, direction, node)

    def combineNode(self, originNode, direction, newNode):
        if direction == nodeDirection.up:
            originNode.up = newNode
            newNode.down = originNode
        elif direction == nodeDirection.down:
            originNode.down = newNode
            newNode.up = originNode
        elif direction == nodeDirection.left:
            originNode.left = newNode
            newNode.right = originNode
        elif direction == nodeDirection.right:
            originNode.right = newNode
            newNode.left = originNode

    def drawNodeMap(self):
        showImage = np.zeros((800, 600, 3), dtype="uint8")
        showImage = showImage + 255
        shape = showImage.shape


        center = (shape[1]/2, shape[0]/6)

        self.nodesDrawn = set()

        self.drawnode(showImage, self.nodeLists[0], center)


        cv2.imshow("Node Lists", showImage)
        #cv2.waitKey()

    def drawnode(self, pic, node, center):

        if node.ID in self.nodesDrawn:
            return 0
        nodeDis = 100

        if node.type == nodeType.corridor:
            colorValue = (0, 0, 0)
        else:
            colorValue = (255, 0, 0)
        cv2.circle(pic, tuple(center), 30, (0, 0, 255))
        cv2.putText(pic, node.name, tuple(center), 0, 0.7, (0, 0, 255))
        self.nodesDrawn.add(node.ID)


        if node.up != None:
            newcenter = (center[0], center[1] -nodeDis)
            if self.drawnode(pic, node.up, newcenter):
                cv2.line(pic, center, newcenter, (0, 0, 0))
        if node.down != None:
            newcenter = (center[0], center[1] +nodeDis)

            if self.drawnode(pic, node.down, newcenter):
                cv2.line(pic, center, newcenter, (0, 0, 0))
        if node.right != None:
            newcenter = (center[0] + nodeDis, center[1])

            if self.drawnode(pic, node.right, newcenter):
                cv2.line(pic, center, newcenter, (0, 0, 0))
        if node.left != None:
            newcenter = (center[0] - nodeDis, center[1])

            if self.drawnode(pic, node.left, newcenter):
                cv2.line(pic, center, newcenter, (0, 0, 0))

        return 1

    def calcNextGoal(self):
        pass

    def setNewPostion(self, newpos):



        self.position = newpos
        self.calcNextGoal()