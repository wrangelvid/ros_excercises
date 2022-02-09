#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import random
import math

def fake_scan_publisher():
  pub = rospy.Publisher('fake_scan', LaserScan, queue_size = 50)
  rospy.init_node('fake_scan_publisher')
  rate = rospy.Rate(20)
  while not rospy.is_shutdown():
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = "base_link"
    scan.angle_min = -(2/3.0)*math.pi
    scan.angle_max = (2/3.0)*math.pi
    scan.angle_increment = math.pi/300.0
    scan.scan_time = 1/20.0
    
    scan.range_min = 1.0
    scan.range_max = 10.0

    scan.ranges = [random.uniform(scan.range_min, scan.range_max) for _ in range(int((scan.angle_max-scan.angle_min)/scan.angle_increment)+1)]
    

    pub.publish(scan)
    rate.sleep()

if __name__ == '__main__':
  try:
    fake_scan_publisher()
  except rospy.ROSInterruptException:
    pass 
