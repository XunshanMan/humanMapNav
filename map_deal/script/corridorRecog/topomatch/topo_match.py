# coding: utf-8
# 程序功能：使用拓扑语义方法进行人类地图与栅格地图之间的匹配

import numpy

# 打算弄个用户界面出来调试如何
from Tkinter import *
# 用于让opencv的图像在Tkinter中实现
from PIL import Image
from PIL import ImageTk

from TopoModule import *

class DataManager:
    def __init__(self):
        # 定义一些变量
        self.flag_choosing_point = False

        # 存放需要match的人类地图和slam地图点
        self.human_points = []
        self.slam_points = []

    def add_human_point(self, point):
        self.human_points.append(point)
        print("Add human_point (%d, %d), total: %d" % (point[0], point[1], len(self.human_points)))

    def add_slam_point(self, point):
        self.slam_points.append(point)
        print("Add slam_point (%d, %d), total: %d" % (point[0], point[1], len(self.slam_points)))

    def is_choosing(self):
        return self.flag_choosing_point

    def change_flag_choosing_point(self):
        self.flag_choosing_point = ~self.flag_choosing_point

    def match(self):


dm = DataManager()


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

    # cv2.drawContours(img, [maxContour], -1, (0, 0, 255), 3)
    rotrect = cv2.minAreaRect(maxContour)
    box = cv2.boxPoints(rotrect)
    box = numpy.int0(box)

    cv2.drawContours(img, [box], 0, (0, 0, 0), 3)

    # def floodFill(image, mask, seedPoint, newVal, loDiff=None, upDiff=None, flags=None): # real signature unknown; restored from __doc__
    h, w = img.shape[:2]
    mask = numpy.zeros((h + 2, w + 2), numpy.uint8)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    # closed = cv2.morphologyEx(img2, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("Close", closed);
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    # cv2.imshow("Open", opened);

    cv2.floodFill(img, mask, seed_pt, (125, 125, 125),
                  (10,) * 3, (10,) * 3, 4)

    mask = cv2.inRange(img, (124, 124, 124), (126, 126, 126))

    cv2.namedWindow("img" + n, cv2.WINDOW_NORMAL)
    # cv2.imshow("img" + n, img)
    cv2.imshow("img" + n, mask)

    return mask


def dealSlamMap(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)

    # cv2.namedWindow("SlamMap", cv2.WINDOW_NORMAL)
    # cv2.imshow("SlamMap", binary)
    return binary


def templatemath_img(image, Target, value):
    imageSrc = image
    template = Target
    w, h = template.shape[:]
    res = cv2.matchTemplate(imageSrc, template, cv2.TM_CCOEFF_NORMED)
    threshold = value
    loc = numpy.where(res >= threshold)

    imgOut = cv2.cvtColor(imageSrc, cv2.COLOR_GRAY2BGR)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(imgOut, pt, (pt[0] + w, pt[1] + h), (7, 249, 151), 2)

    cv2.namedWindow("Detected", cv2.WINDOW_NORMAL)
    cv2.imshow('Detected', imgOut)

    cv2.namedWindow("DetectedRES", cv2.WINDOW_NORMAL)
    cv2.imshow('DetectedRES', res)


def match(slammap, humanmap, slampoint, humanpoint):
    # mt.compareCost()
    cv2.waitKey(0)


def onMouse(event, x, y, flags, param):
    if (event == cv2.EVENT_LBUTTONUP):
        print ("Click.")
        param.setbigstart(x, y)
        param.showRelation()
        param.compareCost()


def readMap():
    # 此处增加对于新主楼的MAP测试
    imgSlam = cv2.imread('/home/lzw/catkin_ws/src/map_deal/script/corridorRecog/gridmap/part_full_startTo.pgm')
    imgSlam_dealt = dealSlamMap(imgSlam)

    # imgHuman = cv2.imread('/home/liaoziwei/pythonProject/corridorRecog/map.jpg')
    # imgHuman_dealt = deal(imgHuman, "- Human", (134, 781))
    # deal函数将人类地图中的走廊提取为二值图，该二值图为inrange函数直接得到
    # 由于之前用另一套算法手动提取了新主楼人类地图的走廊，因此在此处直接读取提取的结果即可
    # 注意直接读取时要按灰度图读取
    # imgHuman_dealt = cv2.imread('/home/lzw/catkin_ws/src/map_deal/script/mapRecognition/pic/map_corridor.jpg', cv2.IMREAD_GRAYSCALE)

    # 经过扩充之后的地图
    imgHuman_dealt = cv2.imread('/home/lzw/catkin_ws/src/map_deal/script/mapRecognition/pic/'
                                'map_corridor_expand.jpg', cv2.IMREAD_GRAYSCALE)

    return imgSlam_dealt, imgHuman_dealt


def startChoseMatchPoint():
    global dm
    dm.change_flag_choosing_point()
    print("当前选择状态:" + str(dm.is_choosing()))


# 初始化用户界面
def initialWindow(root):
    # root.title('controler')
    # root.geometry("550x700")

    # canvas = Canvas(root,width=432,height=700,bg = 'white')
    # canvas.grid(column = 0,row = 0,rowspan=70)
    # im = PhotoImage(file = file_path+"pic.gif")
    # c1=canvas.create_image(0,0,image=im,anchor="nw")

    btn = Button(root, text="testMatch", command=testMatch)
    btn.pack(side="bottom", fill="both", expand="yes", padx="10", pady="10")
    btn = Button(root, text="开始选择匹配点", command=startChoseMatchPoint)
    btn.pack(side="bottom", fill="both", expand="yes", padx="10", pady="10")

    # Button(root, text="visual",width=10,command=active_visual).grid(column = 1,row = 3)
    # Button(root, text="zed",width=10,command=active_zed).grid(column = 1,row = 5)
    # Button(root, text="shot",width=10,command=shot).grid(column = 1,row = 7)
    # Button(root, text="automatic",width=10,command=robocon).grid(column = 1,row = 9)
    # Button(root, text="test_pos+visual",width=10,command=test_pos_visual).grid(column = 1,row = 11)
    # Button(root, text="test_laser",width=10,command=test_laser_communication).grid(column = 1,row = 13)
    # Button(root, text="test_grasp_ready",width=10,command=test_grasp_ready).grid(column = 1,row = 15)
    # Button(root, text="test_grasp_shoot",width=10,command=test_grasp_shoot).grid(column = 1,row = 17)

    # Button(root, text="enable",width=10,command=enable).grid(column = 1,row =19)


# mat : from opencv, Mat
def showImage(panel, mat):
    # convert the images to PIL format...
    image = Image.fromarray(mat)
    # ...and then to ImageTk format
    image = ImageTk.PhotoImage(image)

    if panel is None:
        # the first panel will store our original image
        panel = Label(image=image)
        panel.image = image
        panel.pack(side="left", padx=10, pady=10)


    # otherwise, update the image panels
    else:
        # update the pannels
        panel.configure(image=image)
        panel.image = image


def dealImage():
    mt = mapMatcher(imgSlam_dealt, imgHuman_dealt, start_slam, start_human)
    if mt == None:
        return

    cv2.namedWindow("mapDiff", cv2.WINDOW_NORMAL)
    cv2.createTrackbar('scale', "mapDiff", 100, 200, nothing)
    cv2.createTrackbar('rotate', "mapDiff", initialRotate, 360, nothing)

    switch = '0 : OFF \n1 : ON'
    cv2.createTrackbar(switch, "mapDiff", 0, 1, nothing)

    # automation adjustment
    switch_auto = '0 : AutoA \n1 : AutoB'
    cv2.createTrackbar(switch_auto, "mapDiff", 0, 1, nothing)

    cv2.namedWindow("relationship", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("relationship", onMouse, mt)

    state = 0
    state_auto = 0
    while (1):
        scale = cv2.getTrackbarPos('scale', "mapDiff")
        rotate = cv2.getTrackbarPos('rotate', "mapDiff")
        s = cv2.getTrackbarPos(switch, "mapDiff")
        auto = cv2.getTrackbarPos(switch_auto, "mapDiff")

        if (s != state):
            print ("get on")

            mt.directTransfer(rotate, scale / 100.0)
            mt.showRelation()
            mt.compareCost()
            state = s
        if (auto != state_auto):
            print ("auto get on")

            mt.directTransfer(rotate, scale / 100.0)
            mt.showRelation()
            mt.compareCost()
            mt.autoMatch(3, 100)
            mt.drawAutoMatchResult()
            mt.showRelation(maps[id] + "_" + mt.resize_scale.__str__())

            state_auto = auto

        if cv2.waitKey(30) == "q":
            break

        root.update()

    cv2.destroyAllWindows()


def testMatch():
    print("test match starts.")

# 鼠标回馈函数
def on_mouse_slam(event, x, y, flags, dm):
    if (event == cv2.EVENT_LBUTTONUP):
        if dm.is_choosing():
            dm.add_slam_point((x, y))

def on_mouse_human(event, x, y, flags, dm):
    if (event == cv2.EVENT_LBUTTONUP):
        if dm.is_choosing():
            dm.add_human_point((x, y))

def main():
    root = Tk()
    initialWindow(root)
    # panel = None

    imgSlam_dealt, imgHuman_dealt = readMap()

    # 显示两张地图，并且获得起始位置分别的坐标点

    # 先设定窗口可以自由缩放，不然图片过大
    cv2.namedWindow("slam map", cv2.WINDOW_NORMAL)
    cv2.namedWindow("human map", cv2.WINDOW_NORMAL)

    cv2.imshow("slam map", imgSlam_dealt)
    cv2.imshow("human map", imgHuman_dealt)

    cv2.setMouseCallback("slam map", on_mouse_slam, dm)
    cv2.setMouseCallback("human map", on_mouse_human, dm)

    # cv2.waitKey()

    # templatemath_img(imgSlam_dealt, imgHuman_dealt, 0.5)
    start_slam = [2000, 2000]
    # 实际上为建图时的起始中心

    # 原人类地图走廊参数
    # start_human = [786, 446]
    # 扩展人类地图走廊参数
    start_human = [1123, 941]
    initialRotate = 270

    # 所有处理图像的函数都扔在这里面
    # dealImage()

    # showImage(panel, imgSlam_dealt)

    while True:
        if cv2.waitKey(30) & 0xff == ord('q'):
            break
        root.update()
    root.mainloop()


if __name__ == '__main__':
    main()
