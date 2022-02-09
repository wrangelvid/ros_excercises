#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ros_exercises.msg import OpenSpace
import math

pub_distance = rospy.Publisher('open_space/distance', Float32, queue_size = 10)
pub_angle = rospy.Publisher('open_space/angle', Float32, queue_size = 10)
pub = rospy.Publisher('open_space', OpenSpace, queue_size = 10)

def get_longest_range(msg):
  scan = msg
  longest_range, angle_idx = max(list(zip(scan.ranges, range(len(scan.ranges)))), key = lambda x: x[0])

  angle = scan.angle_min + scan.angle_increment*angle_idx

  rospy.loginfo(Float32(longest_range))
  pub_distance.publish(Float32(longest_range))
  pub_angle.publish(Float32(angle))

  os_data = OpenSpace()
  os_data.angle = angle
  os_data.distance = longest_range
  pub.publish(os_data)


  

  
def listener():
  rospy.init_node('open_space_publisher')
  rospy.Subscriber('fake_scan', LaserScan, get_longest_range)
  rospy.spin()


if __name__ == '__main__':
  listener()
