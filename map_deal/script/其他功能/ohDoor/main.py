import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

from doorRecoger import *

DEBUG_MODE = False


class image_converter(object):

  def __init__(self):

    self.bridge = CvBridge()
    print 'begin listening...'
    rospy.init_node('doorRecoger')
    rospy.Subscriber("cv_camera/image_raw", Image, self.callback)

    # a stack
    self.imageStack = []

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.imageStack.append(cv_image)
    except CvBridgeError as e:
      print(e)

def main(args):
    print 'doorRecoger program v0.1'

    dr = doorRecoger()
    ic = image_converter()

    publisher = rospy.Publisher("isdoor", Bool, queue_size=1)

    while True:
        total = len(ic.imageStack)
        if total < 1:
            continue
        imageNew = ic.imageStack[total - 1]
        #cv2.imshow('NowPic', imageNew)
        imageProcess, isdoor = dr.process(imageNew)
        publisher.publish(isdoor)
        cv2.imshow('ProcessedPic', imageProcess)

        key = cv2.waitKey(30)
        if key is 'q':
            break

    rospy.spin()
    return




if __name__=='__main__':
    if not DEBUG_MODE:
        main(sys.argv)
    else:
        debug()
