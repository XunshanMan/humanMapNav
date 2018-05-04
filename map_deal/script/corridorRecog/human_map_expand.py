#coding:utf-8
from ClassModule import *

def picExpand(matIn):
    size = (2000, 2000)
    matOut = zeros(size)

    # ========================================================================================================
    # 模块说明:
    #       由于OpenCv中,imread()函数读进来的图片,其本质上就是一个三维的数组,这个NumPy中的三维数组是一致的,所以设置图片的
    #   ROI区域的问题,就转换成数组的切片问题,在Python中,数组就是一个列表序列,所以使用列表的切片就可以完成ROI区域的设置
    # ========================================================================================================
    img_roi_height = matIn.shape[0]  # [2]设置ROI区域的高度
    img_roi_width = matIn.shape[1]  # [3]设置ROI区域的宽度
    img_roi_y = size[1]/2 - img_roi_height/2  # [1]设置ROI区域的左上角的起点
    img_roi_x = size[0]/2 - img_roi_width/2


    matOut[img_roi_y:(img_roi_y + img_roi_height), img_roi_x:(img_roi_x + img_roi_width)] = matIn

    cv2.namedWindow("[ROI_Img_DEALT]", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("[ROI_Img_DEALT]", matOut)
    cv2.imwrite("/home/lzw/catkin_ws/src/map_deal/script/mapRecognition/pic/map_corridor_expand.jpg", matOut)
    cv2.waitKey(0)
    cv2.destroyWindow("[ROI_Img_DEALT]")  # [5]python中,创建的窗口,需要手动的销毁

def main():

    picin = cv2.imread("/home/lzw/catkin_ws/src/map_deal/script/mapRecognition/pic/map_corridor.jpg",
                       cv2.IMREAD_GRAYSCALE)

    cv2.namedWindow("picIn", cv2.WINDOW_NORMAL)
    cv2.imshow("picIn", picin)
    cv2.waitKey()

    picExpand(picin)


    print("over")

main()