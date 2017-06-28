#!/usr/bin/env python2

from PubSubManager import *
from juggling_controller.msg import GloveInput
from juggling_swarm_manager.srv import *

from ControlModeHover import ControlModeHover
from ControlModeBTimed import ControlModeBTimed
from ControlModeCircle import ControlModeCircle
from ControlModeBall import ControlModeBall
from ControlModeLand import ControlModeLand
from ControlModeJuggle3D import ControlModeJuggle3D

import numpy as np
import rospy

class objectview(object):
    def __init__(self, d):
        self.__dict__ = d

class Controller(object):
	def __init__(self, swarm_prefix='/swarm/proxy/'):
		self.swarm_prefix = swarm_prefix


		self.last_input = GloveInput()
		self.flying = False
		self.current_trajectory = None

		self.POSSIBLE_TRAJECTORIES = [
			ControlModeHover,
			ControlModeJuggle3D,
			ControlModeCircle,
			ControlModeBTimed,
		]

		self.ball_first_pos_nparr = None


		self.manager = PubSubManager('juggling_controller', anonymous=False, log_level=rospy.DEBUG)

		self.nb_drones = int(self.manager.get_param('~nb_drones', 2))
		self.controller_id = int(self.manager.get_param('~controller_id', 0))
		self.selected_drone = min(self.controller_id, self.nb_drones - 1)

		self.manager.add_client_service(swarm_prefix+'trajectory_manager/load_trajectory',
			Proxy_LoadTrajectory)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/start_trajectory',
			Proxy_Empty)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/stop_trajectory',
			Proxy_Empty)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/visualize',
			Proxy_VisualizeTrajectory)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/next_trajectory',
			Proxy_Empty)

		self.manager.add_client_service(swarm_prefix+'trajectory_manager/trajectory/config_bool',
			Proxy_ConfigTrajectoryBool)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/trajectory/config_float',
			Proxy_ConfigTrajectoryFloat)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/trajectory/config_string',
			Proxy_ConfigTrajectoryString)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/trajectory/config_vector',
			Proxy_ConfigTrajectoryVector)

		self.manager.add_client_service(swarm_prefix+'trajectory_manager/takeoff',
			Proxy_Empty)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/land',
			Proxy_Empty)

		self.manager.add_client_service(swarm_prefix+'trajectory_manager/translate_trajectory',
			Proxy_TranslateTrajectory)
		self.manager.add_client_service(swarm_prefix+'trajectory_manager/set_trajectory_translation',
			Proxy_TranslateTrajectory)

		self.manager.add_client_service(swarm_prefix+'emergency',
			Proxy_Empty)

		self.manager.add_subscriber('input', GloveInput, self._on_glove_input)

	def _on_glove_input(self, input_wrapper):
		glove = input_wrapper['data']

		if glove.glove_id.data != self.controller_id:
			return

		# compute differences from buttons
		button_diff = [int(_a) - int(_b) for _a,_b in zip(glove.buttons, self.last_input.buttons)]

		buttons = {}

		buttons['select_button'] = button_diff[0] # l/r 1
		buttons['action_buttons'] = button_diff[1:5]
		buttons['takeoff_land_button'] = button_diff[5] 
		buttons['translate'] = button_diff[6] # l/r 3
		buttons['emergency'] = button_diff[7] # joystick up
		buttons['prev_drone'] = button_diff[8] # joystick left
		buttons['next_drone'] = button_diff[9] # joystick right

		print buttons

		if buttons['prev_drone'] == 1:
			self._prev_drone()
			print 'SELECTED DRONE :', self.selected_drone

		if buttons['next_drone'] == 1:
			self._next_drone()
			print 'SELECTED DRONE :', self.selected_drone

		if buttons['emergency'] == 1:
			self._call_service('emergency')

		# land/takeoff
		if buttons['takeoff_land_button'] == 1:
			if self.flying:
				self._do_land()
			else:
				self._do_takeoff()

		# trajectory selection
		if 1 in buttons['action_buttons']:
			print 'TRAJECTORY SELECTION'
			trajectory_selected = self.POSSIBLE_TRAJECTORIES[buttons['action_buttons'].index(1)]
			print self._call_service('trajectory_manager/load_trajectory',
				tid=1, trajectory_type=trajectory_selected.get_name())
			self.current_trajectory = trajectory_selected(self)
			print 'TRAJECTORY SELECTED :', self.current_trajectory.get_name()

		if buttons['translate'] == -1:
			vel = vec3_to_tuple(glove.velocity)
			self._call_service('trajectory_manager/translate_trajectory', translation=vel)
			self._call_service('trajectory_manager/visualize', tid=0)

		# trajectory configuration
		if self.current_trajectory is not None:
			self.current_trajectory.on_input(glove, buttons)

		self.last_input = glove


	def _next_trajectory(self):
		self._call_service('trajectory_manager/next_trajectory')
		self._call_service('trajectory_manager/start_trajectory')
		self.current_trajectory = None

		self.ball_first_pos_nparr = None


	def _call_service(self, service, **kwargs):
		print 'CALLING SERVICE', service, kwargs
		if len(kwargs.keys()) > 0:
			res = self.manager.call_service(self.swarm_prefix+service, drone_id=self.selected_drone, payload=objectview(kwargs))
		else:
			res = self.manager.call_service(self.swarm_prefix+service, drone_id=self.selected_drone)
		print 'SERVICE RESULT :', res
		return res

	def _do_land(self):
		if self.flying:
			self._call_service('trajectory_manager/land')
			self.flying = False

	def _do_takeoff(self):
		if not self.flying:
			self._call_service('trajectory_manager/takeoff')
			self.flying = True

	def _prev_drone(self):
		self.selected_drone = (self.selected_drone + self.nb_drones - 1) % self.nb_drones

	def _next_drone(self):
		self.selected_drone = (self.selected_drone + 1) % self.nb_drones



def vec3_to_tuple(v):
	return (v.x, v.y, v.z)
	

if __name__ == '__main__':
	c = Controller()
	c.manager.spin()