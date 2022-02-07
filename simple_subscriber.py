#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import math

pub = rospy.Publisher('random_float_log', Float32, queue_size = 10)

def convert2log(msg):
  num_log = math.log(msg.data)
  rospy.loginfo(num_log)
  pub.publish(Float32(num_log))

  
def listener():
  rospy.init_node('simple_subscriber')
  rospy.Subscriber('my_random_float', Float32, convert2log)
  rospy.spin()


if __name__ == '__main__':
  listener()
