#!/usr/bin/env python2

from visualization_msgs.msg import Marker
from PubSubManager import *
from razer_hydra.msg import Hydra
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Int32
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
import numpy as np

import copy

from juggling_controller.msg import GloveInput

import time

def millis():
	return int(time.time() * 1000.0)

def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

def quat_to_tuple(q):
	return (q.x, q.y, q.z, q.w)

def v3_time_float(vec, f):
	res = Vector3()

	res.x = vec.x * f
	res.y = vec.y * f
	res.z = vec.z * f

	return res

def v3_switch_x_y(vec):
	res = Vector3()
	res.x = -vec.y
	res.y = vec.x
	res.z = vec.z
	return res


class HydraMapper(object):
	def __init__(self):
		self.manager = PubSubManager('hydra_mapper', anonymous=False)
		self.INPUT_ID = int(self.manager.get_param('~input_id', 0))


		self.positions = []

		self.COLOR = [
			ColorRGBA(0, 255, 0, 255),
			ColorRGBA(0, 0, 255, 255)
		]

		self.manager.add_publisher('viz', Marker)
		self.manager.add_publisher('input', GloveInput)

		self.manager.add_subscriber('/hydra_calib', Hydra, self.onHydra)

		self.last_message_sent_millis = 0
		self.framerate = 30


	def fixed_paddle(self, paddle):
		new_paddle = copy.deepcopy(paddle)
		new_paddle.transform.translation = \
			v3_switch_x_y(paddle.transform.translation)
		new_paddle.transform.translation.x += 1.75
		new_paddle.transform.translation.y += 2.25
		new_paddle.transform.translation.z += 1.5
		return new_paddle

	def paddle_to_marker(self, paddle, color, _id):
		m = Marker()

		m.pose.position = paddle.transform.translation
		# m.pose.orientation = Vector3() #paddle.transform.rotation
		m.scale = Vector3(0.1, 0.1, 0.1)
		m.color = color
		m.header = std_msgs.msg.Header()
		m.header.stamp = self.manager.rospy.Time.now()
		m.header.frame_id = '/world'
		m.type = 1
		m.id = _id
		m.lifetime = self.manager.rospy.Duration(10)
		return m


	def create_glove(self, position, rotation, velocity, acceleration, buttons, trigger, joy):
		glove = GloveInput()
		glove.glove_id = Int32(self.INPUT_ID)
		glove.position = Vector3(*position)
		glove.velocity = Vector3(*velocity)
		glove.acceleration = Vector3(*acceleration)
		glove.rotation = Quaternion(*rotation)
		glove.buttons = (
			buttons +
			# [trigger > 0.5] +
			[joy[1] > 0.75] +
			[joy[0] < -0.75] +
			[joy[0] > 0.75]
			)
		return glove

	def onHydra(self, hydra_wrapper):
		h = hydra_wrapper['data']

		paddle = self.fixed_paddle(h.paddles[self.INPUT_ID])

		if self.last_message_sent_millis is 0:
			self.last_message_sent_millis = millis()

		# retrieve positions 
		self.positions.append(np.array(vec3_to_tuple(paddle.transform.translation)))

		# should send a new message ?
		if millis() - self.last_message_sent_millis >= 1000.0 / self.framerate:
			delta_time = float(millis() - self.last_message_sent_millis) / 1000
			
			position = np.array(vec3_to_tuple(paddle.transform.translation))
			rotation = quat_to_tuple(paddle.transform.rotation)

			# computes average velocity since last message
			velocities = [_a-_b for _a,_b in zip(self.positions[1:],self.positions)]
			if len(velocities) == 0:
				velocities.append(np.array([0,0,0]))
			velocities = [v / (delta_time / len(velocities)) for v in velocities]
			velocity = sum(velocities) / len(velocities)

			accelerations = [_a-_b for _a,_b in zip(velocities[1:],velocities)]
			if len(accelerations) == 0:
				accelerations.append(np.array([0,0,0]))
			accelerations = [a / (delta_time / len(accelerations)) for a in accelerations]
			acceleration = sum(accelerations) / len(accelerations)

			self.last_message_sent_millis = millis()
			self.positions = []

			marker = self.paddle_to_marker(paddle, self.COLOR[self.INPUT_ID], 0)

			self.manager.publish('viz', marker)
			self.manager.publish('input',
				self.create_glove(tuple(position), rotation, tuple(velocity), tuple(acceleration),
							 paddle.buttons, paddle.trigger, paddle.joy))


if __name__ == '__main__':
	mapper = HydraMapper()
	mapper.manager.spin()
