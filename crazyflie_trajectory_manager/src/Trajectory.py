class Trajectory(object):
	def __init__(self, manager, **kwargs):
		# Public parameters
		self.values = kwargs
		# special setters
		self.setters = {}
		# started status
		self.started = False
		# trajectory_translation
		self.translation = (0,0,0)

		self.manager = manager

	# Returns a pose
	def get_next_pose(self, drone_position):
		raise NotImplementedError()

	# Returns a tuple of lists of points
	# The first list describes a line
	# The seconds describes an point cloud
	def visualize(self):
		raise NotImplementedError()


	# Start/Stop utilities
	def start(self):
		self.started = True


	def stop(self):
		self.started = False


	# translation utility
	def set_translation(self, t):
		self.translation = t


	# internal functions

	def _set_values(self, **kwargs):
		for k in kwargs:
			self.values[k] = kwargs[k]

	def _get_value(self, key):
		if key in self.values:
			return self.values[key]
		else:
			return None

	def add_setter(self, setter_key, setter_cb):
		self.setters[setter_key] = setter_cb
		return setter_cb



	# Dict-like access to values
	
	# return a list of key strings
	def keys(self):
		return self.values.keys()

	# return a list of (key, item) tuples
	def items(self):
		return self.values.items()

	# return True if the key is here
	def __contains__(self, key):
		return key in self.values

	# return the item named by the key
	def __getitem__(self, key):
		return self._get_value(key)

	# set a value, using special setters if needed
	def __setitem__(self, key, value):
		# if there is a setter, call it
		if key in self.setters:
			self.setters[key](value)
		# else use default setter
		else:
			d = {key: value}
			self._set_values(**d)
			return self.values[key]