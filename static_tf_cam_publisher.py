#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_matrix, euler_matrix,quaternion_from_matrix
import time

def transform2T(msg):
  p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
  q = [msg.rotation.x, msg.rotation.y,msg.rotation.z, msg.rotation.w]
  R = quaternion_matrix(q)
  R[:3,3] = p.T
  return R

def T2Transform(T,parent,child):
  # Initialize a transform object
  transform = geometry_msgs.msg.TransformStamped()
  # Add a timestamp
  transform.header.stamp = rospy.Time.now()
  # Add the source and target frame
  transform.header.frame_id = parent
  transform.child_frame_id =  child
  # Add the translation
  transform.transform.translation.x =  T[0,3]
  transform.transform.translation.y =  T[1,3]
  transform.transform.translation.z =  T[2,3]
  # Add the rotation
  q = quaternion_from_matrix(T)
  transform.transform.rotation.x = q[0]
  transform.transform.rotation.y = q[1]
  transform.transform.rotation.z = q[2]
  transform.transform.rotation.w = q[3]
  
  return transform


class Node(object):
  def __init__(self):
    br1 = tf2_ros.StaticTransformBroadcaster()
    br2 = tf2_ros.StaticTransformBroadcaster()

    T_base_link_gt_left_camera = np.zeros((4,4))
    T_base_link_gt_left_camera[:3,:3] = np.eye(3)
    T_base_link_gt_left_camera[:,3] = np.array([-0.05, 0.0, 0.0, 1.0]).T

    T_base_link_gt_right_camera = np.zeros((4,4))
    T_base_link_gt_right_camera[:3,:3] = np.eye(3)
    T_base_link_gt_right_camera[:,3] = np.array([0.05, 0.0, 0.0, 1.0]).T

    T_left_cam_right_camera = np.linalg.inv(T_base_link_gt_left_camera).dot(T_base_link_gt_right_camera)


    br1.sendTransform(T2Transform(T_base_link_gt_left_camera, 'base_link_gt', 'left_cam'))
    #br2.sendTransform(T2Transform(T_base_link_gt_right_camera, 'base_link_gt', 'right_cam'))


if __name__ == '__main__':
  # Initialize the node
  rospy.init_node("static_tf_cam_publisher")

  node = Node()
  rospy.spin()

