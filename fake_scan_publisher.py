#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import random
import math

def fake_scan_publisher():
  topic = rospy.get_param("/fake_scan_publisher/topic", 'fake_scan')
  pub = rospy.Publisher(topic, LaserScan, queue_size = 50)
  rospy.init_node('fake_scan_publisher')
  publish_rate = rospy.get_param("/fake_scan_publisher/rate",20.0)
  rate = rospy.Rate(publish_rate)
  while not rospy.is_shutdown():
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = "base_link"
    scan.angle_min = rospy.get_param("/fake_scan_publisher/angle_min", -(2/3.0)*math.pi)
    scan.angle_max = rospy.get_param("/fake_scan_publisher/angle_max", (2/3.0)*math.pi)
    scan.angle_increment = rospy.get_param("/fake_scan_publisher/angle_increment", math.pi/300.0)
    scan.scan_time = 1/publish_rate
    
    scan.range_min = rospy.get_param("/fake_scan_publisher/range_min", 1.0)
    scan.range_max = rospy.get_param("/fake_scan_publisher/range_max", 10.0)

    scan.ranges = [random.uniform(scan.range_min, scan.range_max) for _ in range(int((scan.angle_max-scan.angle_min)/scan.angle_increment)+1)]
    

    pub.publish(scan)
    rate.sleep()

if __name__ == '__main__':
  try:
    fake_scan_publisher()
  except rospy.ROSInterruptException:
    pass 
