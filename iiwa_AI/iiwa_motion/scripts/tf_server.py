#!/usr/bin/env python

import rospy
import math
import tf
import time
import geometry_msgs.msg
from motion_msgs.srv import *


global tf_listener
global tf_transformer

def get_transform(parent_frame, child_frame):
	trans = []
	rot = []
	found = False;
	global tf_listener
	if tf_listener.canTransform(parent_frame, child_frame, rospy.Time()):
		try:
			trans, rot = tf_listener.lookupTransform(parent_frame,
					child_frame, rospy.Time())
			found = True
		except (tf.LookupException, tf.ConnectivityException,
				tf.ExtrapolationException):
			rospy.logerr('tf_listener.lookupTransform Failed!!!')
			found = False
	else:
		found = False
	return trans, rot, found


def get_transform_callback(req):
	trans, rot, found = get_transform(req.parent_frame, req.child_frame)
	tr = geometry_msgs.msg.Transform()
	if found:
		tr.translation.x = trans[0]
		tr.translation.y = trans[1]
		tr.translation.z = trans[2]
		tr.rotation.x = rot[0]
		tr.rotation.y = rot[1]
		tr.rotation.z = rot[2]
		tr.rotation.w = rot[3]
	return {'transform': tr, 'success': found}

def transform_pose_callback(req):
	pose_target = geometry_msgs.msg.PoseStamped()
	try:
		pose_target = tf_listener.transformPose(req.target_frame, req.pose)
		success = True
	except (tf.LookupException, tf.ConnectivityException,
			tf.ExtrapolationException):
		rospy.logerr('transform_pose_callback Failed!!!')
		success = False
		pose_target = geometry_msgs.msg.PoseStamped()

	return {'poseWrtTarget': pose_target, 'success': success}	
	
def tf_server():
	# node initallization
	rospy.init_node('tf_server')

	# rename TF Monitor
	global tf_listener
	tf_listener = tf.TransformListener()

	# what is this ??? Coordinate transformation
	# How to use ???  (interpolate=True, cache_time=None)
	global tf_transformer
	tf_transformer = tf.TransformerROS(True, rospy.Duration(10.0))
	
	# rospy service gettransform(customize) and transformpose(Already)
	s = rospy.Service('/tf_server/get_transform', GetTransform,
			get_transform_callback)
	rospy.loginfo("Ready to /tf_server/get_transform")	

	s = rospy.Service('/tf_server/transform_pose', TransformPose,
			transform_pose_callback)
	rospy.loginfo("Ready to /tf_server/transform_pose")	

	# waiting for callback
	rospy.spin()

if __name__ == "__main__":
	tf_server()

