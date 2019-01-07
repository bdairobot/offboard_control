#! /usr/bin/python

import rospy
from mavros_msgs.msg import RCIn
from geometry_msgs.msg import PoseStamped
from offboard_control.msg import Path
import math

class Generator(object):
  def __init__(self):
    rospy.init_node("backtracking_generator")
    pose_topic_sub = rospy.get_param("~pose_topic_sub","/mavros/local_position/pose")
    rc_topic_sub = rospy.get_param("~rc_topic_sub", "/mavros/rc/in")
    path_topic_pub = rospy.get_param("~path_topic_pub", "/mavros/desired_path")

    self.pose_sub = rospy.Subscriber(pose_topic_sub, PoseStamped, self.pose_cb)
    self.rc_sub = rospy.Subscriber(rc_topic_sub, RCIn, self.rc_cb)
    self.path_pub = rospy.Publisher(path_topic_pub, Path, queue_size=1)
    self.path = Path()
    self.record_flag = 0
    self.flag_channel = 6
    self.min_distance = 1
    self.published = 0
  
  def rc_cb(self, msg):
    rc_value = msg.channels[self.flag_channel]
    if rc_value>=1750:
      self.record_flag = 2
    elif 1250<rc_value and rc_value < 1750:
      self.record_flag = 1
    else: self.record_flag = 0

  def pose_cb(self, msg):
    if self.record_flag == 1:
      self.published = 0
      if not self.path.waypoints:
        self.path.waypoints.append(msg)
	rospy.loginfo("OffboardGenerator: Start record position...")
      elif math.sqrt(math.pow(msg.pose.position.x-self.path.waypoints[-1].pose.position.x,2) + 
        math.pow(msg.pose.position.y-self.path.waypoints[-1].pose.position.y,2) + 
        math.pow(msg.pose.position.z-self.path.waypoints[-1].pose.position.z,2)) > self.min_distance:
        self.path.waypoints.append(msg)

    elif self.record_flag == 2:
      if len(self.path.waypoints) > 1 and not self.published:
	self.path.waypoints.reverse()
        self.path_pub.publish(self.path)
        rospy.loginfo("Published path length: %d" % len(self.path.waypoints))
        self.published = 1
    else: 
      self.published = 0
      self.path.waypoints = []

if __name__ == '__main__':
  path_generator = Generator()
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    rate.sleep()
  rospy.spin
