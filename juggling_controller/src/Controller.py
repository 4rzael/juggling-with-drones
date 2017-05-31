#!/usr/bin/env python2

from PubSubManager import *
from juggling_controller.msg import GloveInput
from juggling_swarm_manager.srv import *

from ControlModeHover import ControlModeHover
from ControlModeBTimed import ControlModeBTimed
from ControlModeCircle import ControlModeCircle
from ControlModeBall import ControlModeBall
from ControlModeLand import ControlModeLand

import numpy as np
import rospy

class objectview(object):
    def __init__(self, d):
        self.__dict__ = d

class Controller(object):
	def __init__(self, swarm_prefix='/swarm/proxy/'):
		self.swarm_prefix = swarm_prefix

		self.manager = PubSubManager('juggling_controller', anonymous=False, log_level=rospy.DEBUG)

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
		self.selected_drone = int(self.manager.get_param('~controller_id', 0))

		self.last_input = GloveInput()
		self.flying = False
		self.current_trajectory = None

		self.POSSIBLE_TRAJECTORIES = [
			ControlModeHover,
			ControlModeLand,
			ControlModeCircle,
			ControlModeBall,
		]

		self.ball_first_pos_nparr = None

	def _on_glove_input(self, input_wrapper):
		glove = input_wrapper['data']

		self.manager.rospy.logdebug(glove)

		# compute differences from buttons
		button_diff = [int(_a) - int(_b) for _a,_b in zip(glove.buttons, self.last_input.buttons)]

		buttons = {}

		buttons['action_buttons'] = button_diff[1:5]
		buttons['select_button'] = button_diff[0] # l/r 1
		buttons['takeoff_land_button'] = button_diff[5] 
		buttons['start_button'] = button_diff[7] # trigger
		buttons['translate'] = button_diff[6] # l/r 3
		buttons['emergency'] = button_diff[8] # joystick Y

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
			self._call_service('trajectory_manager/load_trajectory',
				tid=1, trajectory_type=trajectory_selected.get_name())
			self.current_trajectory = trajectory_selected(self)
			print 'TRAJECTORY SELECTED :', self.current_trajectory.get_name()

		# trajectory submitting and start
		if buttons['start_button'] == 1:
			self._call_service('trajectory_manager/next_trajectory')
			self._call_service('trajectory_manager/start_trajectory')
			self.current_trajectory = None

			self.ball_first_pos_nparr = None

		if buttons['translate'] == -1:
			vel = vec3_to_tuple(glove.velocity)
			self._call_service('trajectory_manager/translate_trajectory', translation=vel)
			self._call_service('trajectory_manager/visualize', tid=0)

		# trajectory configuration
		if self.current_trajectory is not None:
			self.current_trajectory.on_input(glove, buttons)

		self.last_input = glove


	def _call_service(self, service, **kwargs):

		if len(kwargs.keys()) > 0:
			self.manager.call_service(self.swarm_prefix+service, drone_id=self.selected_drone, payload=objectview(kwargs))
		else:
			self.manager.call_service(self.swarm_prefix+service, drone_id=self.selected_drone)

	def _do_land(self):
		if self.flying:
			self._call_service('trajectory_manager/land')
			self.flying = False

	def _do_takeoff(self):
		if not self.flying:
			self._call_service('trajectory_manager/takeoff')
			self.flying = True



def vec3_to_tuple(v):
	return (v.x, v.y, v.z)
	

if __name__ == '__main__':
	c = Controller()
	c.manager.spin()