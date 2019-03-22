#! /usr/bin/python

import rospy
from mavros_msgs.msg import RCIn
from geometry_msgs.msg import Quaternion, PoseStamped

import actionlib
import offboard_control.msg
import math
import numpy
import tf

class Generator(object):
  def __init__(self):
    rospy.init_node("backtracking_generator")
    pose_topic_sub = rospy.get_param("~pose_topic_sub","/mavros/local_position/pose")
    rc_topic_sub = rospy.get_param("~rc_topic_sub", "/mavros/rc/in")
    action_server_name = rospy.get_param("~action_server_name","/move_base")

    self.ac = actionlib.SimpleActionClient(action_server_name, offboard_control.msg.PathNavigationAction)

    self.pose_sub = rospy.Subscriber(pose_topic_sub, PoseStamped, self.pose_cb)
    self.rc_sub = rospy.Subscriber(rc_topic_sub, RCIn, self.rc_cb)
    self.path = []
    self.record_flag = 0
    self.flag_channel = 6
    self.min_distance = 0.5
    self.published = 0
  
  def rc_cb(self, msg):
    rc_value = msg.channels[self.flag_channel]
    if rc_value>=1750:
      self.record_flag = 2
    elif 1250<rc_value and rc_value < 1750:
      self.record_flag = 1
    else: self.record_flag = 0

  def pose_cb(self, msg):
    def rotate_q(q):
      """
      Rotation for 180 degrees
      """
      dq = tf.transformations.quaternion_from_euler(0, 0, math.radians(180))
      new_q = tf.transformations.quaternion_multiply(dq,numpy.array([q.x,q.y,q.z,q.w]))
      return Quaternion(new_q[3],new_q[0],new_q[1],new_q[2])

    if self.record_flag == 1:
      self.published = 0
      if not self.path:
        msg.pose.orientation = rotate_q(msg.pose.orientation)
        self.path.append(msg)
        rospy.loginfo("OffboardGenerator: Start record position...")
      elif math.sqrt(math.pow(msg.pose.position.x-self.path[-1].pose.position.x,2) + 
        math.pow(msg.pose.position.y-self.path[-1].pose.position.y,2) + 
        math.pow(msg.pose.position.z-self.path[-1].pose.position.z,2)) > self.min_distance:
        msg.pose.orientation = rotate_q(msg.pose.orientation)
        self.path.append(msg)

    elif self.record_flag == 2:
      if len(self.path) > 1 and not self.published:
        self.path.reverse()
        goal = offboard_control.msg.PathNavigationGoal(path=self.path)
        self.ac.send_goal(goal)
        rospy.loginfo("OffboardGenerator: Published path length: %d" % len(self.path))
        self.published = 1
        # self.ac.wait_for_result()
        # rospy.loginfo("OffboardGenerator: Result:" + self.ac.get_result())
    else: 
      self.published = 0
      self.path = []

if __name__ == '__main__':
  path_generator = Generator()
  rate = rospy.Rate(10)
  path_generator.ac.wait_for_server()
  while not rospy.is_shutdown():
    rate.sleep()
  rospy.spin
