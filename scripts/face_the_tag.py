#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg  import Twist
from apriltags2_ros.msg import AprilTagDetectionArray

class face_the_tag:

  def __init__(self):
    self.params()
    self.pubs()
    self.subs()
    self.detection_timeout = rospy.get_rostime()
    rospy.Timer( rospy.Duration( 1/self.rate ), self.timerCallback )

  def params(self):
    self.target_id = rospy.get_param('~target_id', 'none')
    self.lin_vel   = rospy.get_param('~lin_vel',  0.0)
    self.ang_vel   = rospy.get_param('~ang_vel',  0.3)
    self.rate      = rospy.get_param('~rate'   , 10.0)
    self.timeout   = rospy.get_param('~timeout',  3.0)

  def subs(self):
    rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagCallback)

  def pubs(self):
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

  def tagCallback(self, tag_detections_array):
    for detection in tag_detections_array.detections:
      print("Theres a detection")
      print(self.target_id)
      tag_id = detection.id[0]
      print(tag_id)
      if not tag_id == self.target_id:
        continue
      print("Found what I'm looking for!")
      # Z forward, X right
      z = detection.pose.pose.pose.position.z
      x = detection.pose.pose.pose.position.x
      ang = math.atan2( x, z )
      vel = -ang*1;
      if abs(vel) > self.ang_vel:
        vel = self.ang_vel * (vel / abs(vel))
      print("vel: ")
      print(vel)
      self.vel = vel
      self.detection_timeout = rospy.get_rostime() + rospy.Duration( self.timeout )

  def timerCallback(self, event):
    if rospy.get_rostime() > self.detection_timeout:
      vel = 0
    else:
      vel = self.vel
    msg = Twist()
    msg.angular.z = vel
    self.pub.publish(msg)
    #print("Publishing")

if __name__ == '__main__':
  print("Initialising node")
  rospy.init_node('face_the_tag', anonymous=True)
  print("Creating object")
  obj = face_the_tag()
  
  
  rospy.spin()
