from ControlModeBSpline import ControlModeBSpline

def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

class ControlModeBTimed(ControlModeBSpline):
	@staticmethod
	def get_name():
		return 'btimed'

	def on_input(self, glove, buttons_diff):
		if buttons_diff['select_button'] == -1:
			self.send()

		if buttons_diff['select_button'] == 1:
			self.call_service('trajectory_manager/trajectory/config_float',
				tid=1, key='speed', value=0.5)
		super(ControlModeBTimed, self).on_input(glove, buttons_diff)
