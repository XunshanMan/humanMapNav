from ClassModule import *

def main():

    picin = cv2.imread("/home/liaoziwei/Desktop/runSpace/searchGoalCalcu/waitForDebug/crash.jpg")

    cv2.namedWindow("picIn", cv2.WINDOW_NORMAL)
    cv2.imshow("picIn", picin)

    graypic = cv2.cvtColor(picin, cv2.COLOR_BGR2GRAY)

    points,theta,flag = fromLineGetSearchPoints_web(graypic, 25, 5)

    print("over")

main()