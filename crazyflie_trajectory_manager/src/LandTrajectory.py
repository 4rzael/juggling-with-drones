from Trajectory import Trajectory
from geometry_msgs.msg import Pose, Vector3
from TrajectoryUtils import tuple3_add, millis

# Public arguments :
# * position : The position the drone should land at : (x, y, 0)
# * viz_position : The drone position used for visualisation purposes

# Does not care about translation
class LandTrajectory(Trajectory):
	def __init__(self, position=None, viz_position=None, **kwargs):
		super(LandTrajectory, self).__init__(**kwargs)
		self.add_setter('position', self.set_position)

		if position is not None:
			self['position'] = position
		if viz_position is not None:
			self['viz_position'] = viz_position

		self.first_time = True
		self.last_point_millis = 0

		self.points = []
		self.current_index = 0

	def set_position(self, pos):
		if 'viz_position' not in self:
			self['viz_position'] = pos

		pos = (pos[0], pos[1], 0)
		self._set_values(position=pos)

	def _compute(self, drone_position):
		# compute points with z decreasing from the first position to 0.5 meters
		return [(self['position'][0], self['position'][1], float(z) / 10.0)
			for z in sorted(range(10, int(drone_position[2]*10)), reverse=True)]

	def get_next_pose(self, drone_position):
		if 'position' not in self:
			self['position'] = drone_position

		if self.first_time:
			self.last_point_millis = millis()
			self.first_time = False
			self.points = self._compute(drone_position)

		pose = Pose()

		if millis() - self.last_point_millis >= 200: # 0.5m/s
			self.current_index += 1
			self.last_point_millis = millis()

		if self.current_index < len(self.points):
			pose.position = Vector3(*self.points[self.current_index])
		else: # our Z position is great, activate landing mode 
			self.manager.land()
			pose.position = Vector3(*self['position'])

		return pose

	def visualize(self, **kwargs):
		print self['viz_position']

		if not self.first_time:
			points = self.points
		else:
			if 'viz_position' in self:
				pos = self['viz_position']
				points = self._compute(pos) + [self['position']]
			else:
				points = [self['position']]

		return (points, [self['position']])
