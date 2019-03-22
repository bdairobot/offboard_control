import rospy
from geometry_msgs.msg import Quaternion,PoseStamped
import tf
import math
import numpy

def pose_cb(msg):
    dq = tf.transformations.quaternion_from_euler(0, 0, math.radians(180))
    q = msg.pose.orientation
    new_q = tf.transformations.quaternion_multiply(dq,numpy.array([q.x,q.y,q.z,q.w]))
    return Quaternion(new_q[3],new_q[0],new_q[1],new_q[2])

if __name__ == "__main__":
    rospy.init_node("tester")
    pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.spin()