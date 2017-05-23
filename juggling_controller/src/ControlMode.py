class ControlMode(object):
	def __init__(self, call_service):
		self.call_service = call_service
		pass

	# static
	@staticmethod
	def get_name():
		raise NonImplementedError()

	def on_input(self, glove, buttons_diff):
		raise NonImplementedError()

