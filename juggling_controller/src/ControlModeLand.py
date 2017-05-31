from ControlMode import ControlMode

def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

class ControlModeLand(ControlMode):
	@staticmethod
	def get_name():
		return 'land_at_pos'

	def on_input(self, glove, buttons_diff):
		if buttons_diff['select_button'] == 1:
			pt = vec3_to_tuple(glove.position)
			pt = (1.5,1.5,2)
			self.call_service('trajectory_manager/trajectory/config_vector',
				tid=1, key='position', value=pt)
			self.call_service('trajectory_manager/visualize',
				tid=1)
