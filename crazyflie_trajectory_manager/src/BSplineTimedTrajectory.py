from Trajectory import Trajectory
from TrajectoryUtils import distance, compute_trajectory_distance
from BSplineTrajectory import BSplineTrajectory, _need_compute

from geometry_msgs.msg import Pose, Vector3

import time
def millis():
	return int(time.time() * 1000.0)


# Public arguments :
# - See BSplineTrajectory arguments
# * trajectory_time : The time taken to do the full trajectory : 0.0f -> Inf

class BSplineTimedTrajectory(BSplineTrajectory):
	def __init__(self, trajectory_time=None, **kwargs):
		super(BSplineTimedTrajectory, self).__init__(**kwargs)

		self['trajectory_time'] = trajectory_time
		# the speed is the fallback for the trajectory_time.
		# If no trajectory_time is given, it will be computed from the speed
		self['speed'] = 0.5 # in m/s
		self.last_point_time = 0
		self.is_first_pos = True

	@_need_compute
	def _get_trajectory_distance(self):
		return compute_trajectory_distance(self.computed_points, loop=self['loop'])

	@_need_compute
	def get_next_pose(self, drone_position):
		if self.is_first_pos:
			self.is_first_pos = False
			self.last_point_time = millis()
		p = Pose()

		if self['trajectory_time'] is not None:
			trajectory_time = self['trajectory_time']
		else:
			# compute time from trajectory distance and speed
			trajectory_time = self['speed'] * self._get_trajectory_distance()


		# if timeout passed, go to the next point
		if float(millis() - self.last_point_time) / 1000 >= trajectory_time / len(self.computed_points):
			super(BSplineTimedTrajectory, self)._nextPoint()
			self.last_point_time = millis()
		else: # else, use the underlying trajectory mode : point-to-point
			current_pose = super(BSplineTimedTrajectory, self).get_current_pose()
			next_pose = super(BSplineTimedTrajectory, self).get_next_pose(drone_position)
			if current_pose != next_pose:
				self.last_point_time = millis()				

		p.position = Vector3(*self.computed_points[self.current_point_index])
		return p
