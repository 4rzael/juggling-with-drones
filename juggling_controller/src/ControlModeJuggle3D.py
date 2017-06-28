from ControlMode import ControlMode
import numpy as np

def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

class ControlModeJuggle3D(ControlMode):
	def __init__(self, *args, **kwargs):
		super(ControlModeJuggle3D, self).__init__(*args, **kwargs)
		self.first_pos = None

	@staticmethod
	def get_name():
		return 'juggle3D'

	def on_input(self, glove, buttons_diff):
		if buttons_diff['select_button'] == 1:
			self.first_pos = np.array(vec3_to_tuple(glove.position))

		if buttons_diff['select_button'] == -1:
			position = np.array(vec3_to_tuple(glove.position))
			vec = position - self.first_pos
			speed = vec3_to_tuple(glove.velocity)
			speed = [s * 2 for s in speed]

			self.call_service('trajectory_manager/trajectory/config_vector',
				tid=1, key='hand_vector', value=speed)
			self.call_service('trajectory_manager/trajectory/config_vector',
				tid=1, key='base_x_vector', value=(1,0,0))
			self.call_service('trajectory_manager/trajectory/config_float',
				tid=1, key='time_multiplier', value=0.5)
			self.call_service('trajectory_manager/trajectory/config_vector',
				tid=1, key='init_speed', value=speed)
			self.call_service('trajectory_manager/visualize',
				tid=1)
			self.send()
