from ControlMode import ControlMode

def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

class ControlModeHover(ControlMode):
	@staticmethod
	def get_name():
		return 'hover'

	def on_input(self, glove, buttons_diff):
		if buttons_diff['select_button'] == 1:
			hover_point = vec3_to_tuple(glove.position)
			self.call_service('trajectory_manager/trajectory/config_vector',
				tid=1, key='position', value=hover_point)
			self.call_service('trajectory_manager/visualize',
				tid=1)

