from ControlMode import ControlMode

def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

class ControlModeBall(ControlMode):
	@staticmethod
	def get_name():
		return 'ball'

	def on_input(self, glove, buttons_diff):
		if buttons_diff['select_button'] == -1:

			speed = vec3_to_tuple(glove.velocity)
			speed = [s * 2 for s in speed]
			self.call_service('trajectory_manager/trajectory/config_vector',
				tid=1, key='init_speed', value=speed)
			self.call_service('trajectory_manager/trajectory/config_float',
				tid=1, key='time_multiplier', value=0.5)
			self.call_service('trajectory_manager/visualize',
				tid=1)

