from geometry_msgs.msg import PoseStamped
from tf import TransformListener
from tf2_ros import TransformException
import rospy

def transformPoint(listener, base_link_point):


    base_link = PoseStamped()
    base_link.header.frame_id = "base_link"
    base_link.header.stamp=rospy.Time()

    base_link.pose.position.x=base_link_point.pose.position.x
    base_link.pose.position.y=base_link_point.pose.position.y
    base_link.pose.position.z=0

    base_link.pose.orientation.x = base_link_point.pose.orientation.x
    base_link.pose.orientation.y = base_link_point.pose.orientation.y
    base_link.pose.orientation.z = base_link_point.pose.orientation.z
    base_link.pose.orientation.w = base_link_point.pose.orientation.w


    map_point = PoseStamped()
    now = rospy.Time.now()
    listener.waitForTransform("base_link", "map",
                              now, rospy.Duration(3.0));
    #listener.lookupTransform("/turtle2", "/turtle1",
    #                         now, transform);
    map_point = listener.transformPose("map", base_link)

    #print("base_link:(%.2f, %.2f, %.2f) -----> base_link:(%.2f, %.2f, %.2f) at time %.2f" %(base_link.pose.orientation.x,
    #         base_link.pose.orientation.y, base_link.pose.orientation.z, map_point.pose.orientation.x, map_point.pose.orientation.y, map_point.pose.orientation.z,
    #         map_point.header.stamp.to_time()));

    return map_point

# As what we need is (0,0) Point of base_link. So we don't need a base_link_point.
# 1.17  finally work out. cheers!!!
def transformPoint_new(listener, base_link_point):

    t = listener.getLatestCommonTime("/base_link", "/map")

    if t:

        #position, quaternion = listener.lookupTransform("/base_link", "/map", t)
        #print ("Succeed in getting transform: ")
        #print position, quaternion

        base_link = PoseStamped()
        base_link.header.frame_id = "base_link"
        base_link.header.stamp = rospy.Time()

        map_point = PoseStamped()

        try:
            listener.waitForTransform("base_link", "map",
                                      t, rospy.Duration(3.0));
            # listener.lookupTransform("/turtle2", "/turtle1",
            #                         now, transform);
            map_point = listener.transformPose("map", base_link)

            # print("base_link:(%.2f, %.2f, %.2f) -----> map_point:(%.2f, %.2f, %.2f) at time %.2f" % (
            #     base_link.pose.position.x,
            #     base_link.pose.position.y, base_link.pose.position.z, map_point.pose.position.x,
            #     map_point.pose.position.y, map_point.pose.position.z,
            #     map_point.header.stamp.to_time()));
        except TransformException, e:
            print("Exeption: %s" % e.message)


        return map_point
    else:
        print("tf transformation does't exists. tfs:")
        for i in listener.getFrameStrings():
            print i
        map_point = PoseStamped()

        return map_point
