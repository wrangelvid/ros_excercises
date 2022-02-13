#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_matrix, euler_matrix,quaternion_from_matrix

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


rospy.init_node("base_link_tf_pub")
# Initialize the node
br = tf2_ros.TransformBroadcaster()

# Publish messages at 10 hz
r = rospy.Rate(20)

T_base_link_gt_left_camera = np.zeros((4,4))
T_base_link_gt_left_camera[:3,:3] = np.eye(3)
T_base_link_gt_left_camera[:,3] = np.array([-0.05, 0.0, 0.0, 1.0]).T


# Create a listener
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

while not rospy.is_shutdown():
  # Fetch the transform
  try:
    transform = tfBuffer.lookup_transform('world', 'left_cam', rospy.Time())
  except:
    continue
  #convert to 4x4 array
  T_W_left_camera = transform2T(transform.transform)

  T_base_link_gt_world = T_W_left_camera.dot(np.linalg.inv(T_base_link_gt_left_camera))

  br.sendTransform(T2Transform(T_base_link_gt_world, 'world', 'base_link_gt2'))

  # Wait for the next event
  r.sleep()
