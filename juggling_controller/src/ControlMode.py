class ControlMode(object):
	def __init__(self, controller):
		self.controller = controller
		self.call_service = controller._call_service
		self.send = controller._next_trajectory
		pass

	# static
	@staticmethod
	def get_name():
		raise NonImplementedError()

	def on_input(self, glove, buttons_diff):
		raise NonImplementedError()

