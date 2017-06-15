from Trajectory import Trajectory
from TrajectoryUtils import delete_useless_points, returns_pose, millis

import numpy as np

# Public arguments :
# - init_speed : The initial speed : (x, y, z)
# - init_pos : The initial position. Will start from the drone position if not given : (x, y, z)
# - time_multiplier : time multiplier : 0.01 -> 100

class BallLikeTrajectory(Trajectory):
	def __init__(self, init_speed=(0.0,0.0,0.0), init_pos=None,
		time_multiplier=0.75, **kwargs):

		super(BallLikeTrajectory, self).__init__(**kwargs)

		self['init_speed'] = init_speed
		self['init_pos'] = init_pos
		self['time_multiplier'] = time_multiplier
		self['gravity'] = (0.0, 0.0, -9.81)

		self['viz_pos'] = (2.0,2.0,1.0)

		self.first_call = True
		self.last_point_millis = 0

		self.computed_points = None
		self.current_point_index = 0


	def _should_stop(self, point, init_pos):
		return point[2] < init_pos[2]

	def _compute(self, options):
		P = np.array(options['init_pos'])
		V = np.array(options['init_speed'])
		A = np.array(options['gravity'])


		pts = [options['init_pos']]

		for t in range (0, 10*1000, 100):
			if self._should_stop(P, options['init_pos']):
				break
			V += A * 0.1
			P += V * 0.1
			pts.append(tuple(P))

		return pts

	def compute_points(self):
		self.computed_points = self._compute(self)

	def _nextPoint(self):
		self.current_point_index = min(self.current_point_index + 1, len(self.computed_points) - 1)
		self.last_point_millis = millis()

	@returns_pose
	def get_next_pose(self, drone_position):
		if not self.started:
			return drone_position

		if self.first_call:
			self.last_point_millis = millis()
			if self['init_pos'] is None:
				self['init_pos'] = drone_position
			self.first_call = False

		if self.computed_points is None:
			self.compute_points()

		if millis() - self.last_point_millis >= (100.0 / self['time_multiplier']):
			self._nextPoint()
		return self.computed_points[self.current_point_index]

	def visualize(self):
		options = {
				'init_pos': self['viz_pos'],
				'init_speed': self['init_speed'],
				'gravity': self['gravity']
			}

		line = self._compute(options)

		return (
			line,
			[options['init_pos'], (_p+_s for _p,_s in zip(options['init_pos'], options['init_speed']))]
			)
