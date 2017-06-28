from ControlMode import ControlMode

def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

class ControlModeBSpline(ControlMode):
	@staticmethod
	def get_name():
		return 'bspline'

	def on_input(self, glove, buttons_diff):
		if buttons_diff['select_button'] == -1:
			self.send()

		if buttons_diff['select_button'] == 1:
			new_point = vec3_to_tuple(glove.position)
			self.call_service('trajectory_manager/trajectory/config_bool',
				tid=1, key='loop', value=True)
			self.call_service('trajectory_manager/trajectory/config_vector',
				tid=1, key='add_point', value=new_point)
			self.call_service('trajectory_manager/visualize',
				tid=1)
