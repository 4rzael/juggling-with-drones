import json

import rospy
from PubSubManager import PubSubManager
from geometry_msgs.msg import * 
from juggling_controller.msg import GloveInput
from std_msgs.msg import *

import numpy as np

import sys

def vec3_to_nparray(v):
	return np.array((v.x, v.y, v.z))

class LowPassFilter(object): # weighted average filter
	def __init__(self, filter_delay=5, np_threshold=0.25):
		self.values = []
		self.filter_delay = filter_delay
		self.np_threshold = np_threshold

	def compute_filter(self, array):
		if len(self.values) is 0:
			return np.array(0.0, 0.0, 0.0)
		return reduce(lambda a,b: a+b, self.values) / len(self.values)

	def apply_threshold(self, nparray):
		def t(v):
			return v if abs(v) >= self.np_threshold else 0
		return np.array(map(t, list(nparray)))

	def __call__(self, value):
		value = self.apply_threshold(value)

		self.values.append(value)
		if len(self.values) > self.filter_delay:
			self.values = self.values[1:]
		return self.compute_filter(self.values)


position = np.array((0.0,0.0,0.0))
velocity = np.array((0.0,0.0,0.0))
acceleration = np.array((0.0,0.0,0.0))
rotation = None
last_pose_micros_tms = None
last_pose = None

acc_filter = LowPassFilter(10)

manager = PubSubManager('phone_glove', anonymous=False)

def stamp_to_micros(stamp):
	return stamp.secs * 10e6 + stamp.nsecs * 10e-3


def on_pose(wrapped_pose):
	global position
	global velocity
	global acceleration
	global last_pose_micros_tms
	global last_pose

	pose = wrapped_pose['data'].pose
	tms = stamp_to_micros(wrapped_pose['data'].header.stamp)

	position = vec3_to_nparray(pose.position)
	rotation = pose.orientation
	if last_pose is None:
		last_pose = pose
		velocity = np.array((0.0, 0.0, 0.0))
		acceleration = np.array((0.0, 0.0, 0.0))
	else:
		last_velocity = velocity
		velocity = (position - vec3_to_nparray(last_pose.position)) / (tms - last_pose_micros_tms)
		acceleration = (velocity - last_velocity) / (tms - last_pose_micros_tms)

	last_pose = pose
	last_pose_micros_tms = tms


def obj_to_glove_input(obj):
	print obj
	global manager
	global position
	global velocity
	global acceleration
	global rotation

	glove = GloveInput()
	glove.glove_id = Int32(int(obj['controller_id']))
	glove.position = Vector3(*position)
	glove.velocity = Vector3(*velocity)
	glove.acceleration = Vector3(*acceleration)
	glove.rotation = rotation if rotation is not None else Quaternion(0,0,0,0)
	glove.buttons = (
		[obj['big_button']] + # big button
		obj['trajectory_selections'] +
		[obj['abort'], # emergency
		0, # prev
		0, # next
		])
	print glove

	manager.publish('input', glove)

manager.add_subscriber('pose', PoseStamped, on_pose)
manager.add_publisher('input', GloveInput)


if __name__ == '__main__':
	print 'python ROS bridge started successfully'
	for line in sys.stdin:
		try:
			obj = json.loads(line)
		except:
			obj = None
		if obj is not None:
			try:
				obj_to_glove_input(obj)
			except Exception as e:
				print e
				raise
