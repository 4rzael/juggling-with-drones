from Trajectory import Trajectory
from TrajectoryUtils import distance, delete_useless_points, tuple3_add

from scipy import interpolate
import numpy as np

from geometry_msgs.msg import Pose, Vector3


def _need_compute(method):
	def wrapper(self, *args, **kwargs):
		if not self.is_computed:
			try:
				self.compute_points()
			except Exception as e:
				print 'ERROR in _need_compute', e
				raise
		return method(self, *args, **kwargs)
	return wrapper


# Public arguments :
# * key_points : The points the trajectory should go through : [(x, y, z), ...]
# * poly_order : Desired polynomial order of spline curves : 1 -> 5
# * loop : Should the trajectory loop : True/False
# * closed : Is the trajectory closed. True if loop=True : True/False
# * precision : Precision of the trajectory in meters

class BSplineTrajectory(Trajectory):
	def __init__(self, key_points=[], poly_order=3, precision=0.1, loop=False, closed=False, **kwargs):
		super(BSplineTrajectory, self).__init__(**kwargs)

		self.add_setter('key_points', self.set_points)
		self.add_setter('poly_order', self.set_poly_order)
		self.add_setter('loop', self.set_loop_mode)
		self.add_setter('closed', self.set_closed_mode)
		self.add_setter('precision', self.set_precision)
		# hacky way to add points
		self.add_setter('add_point', self.add_point)

		self['key_points'] = key_points
		self['poly_order'] = poly_order
		self['precision'] = precision
		self['closed'] = closed # must be before assigning 'loop'
		self['loop'] = loop
		self.computed_points = None

		self.is_computed = False


	def add_point(self, p):
		self['key_points'] = self['key_points'] + [p]


	def compute_points(self):
		key_points = [tuple3_add(p, self.translation) for p in self['key_points']]

		if self['closed'] and len(self['key_points']) > 0:
			# if need to loop, add a useless waypoint, because
			# the last one is not taken in account by splprep
			to_add_point = map(lambda p: 42.42, key_points[0])
			key_points = key_points + [to_add_point]

		rotated = list(zip(*key_points))
		tck,u=interpolate.splprep(rotated, k=self.poly_order_reached, s=0, per=self['closed'])
		u=np.linspace(0,1,num=100,endpoint=True)
		out = interpolate.splev(u,tck)
		self.computed_points = delete_useless_points(list(zip(*out)), self['precision'])
		self.current_point_index = 0
		self.is_computed = True
		self.current_pose = Pose()
		self.current_pose.position = Vector3(*self.computed_points[0])

	@_need_compute
	def get_current_pose(self):
		return self.current_pose


	@_need_compute
	def get_next_pose(self, drone_position):
		p = Pose()
		if distance(drone_position, self.computed_points[self.current_point_index]) <= self['precision']:
			self._nextPoint()
		p.position = Vector3(*self.computed_points[self.current_point_index])
		self.current_pose = p
		return p


	@_need_compute
	def _nextPoint(self):
		if self['loop']:
			self.current_point_index = (self.current_point_index + 1) % len(self.computed_points)
		else:
			self.current_point_index = min(self.current_point_index + 1, len(self.computed_points) - 1)


	@_need_compute
	def visualize(self):
		print self['key_points']
		return (self.computed_points, self['key_points'])


	def set_points(self, pts):
		self._set_values(key_points=pts)
		self.set_poly_order(self['poly_order'])
		self.is_computed = False

	def set_poly_order(self, poly_order):
		self._set_values(poly_order=poly_order)
		self.poly_order_reached = min(len(self['key_points']) - 1, poly_order)
		self.is_computed = False

	def set_precision(self, precision):
		self._set_values(precision=precision)
		self.is_computed = False

	def set_loop_mode(self, loop_mode):
		self._set_values(loop=loop_mode)
		if loop_mode:
			self['closed'] = True
		self.is_computed = False

	def set_closed_mode(self, closed_mode):
		self._set_values(closed=closed_mode)
		self.is_computed = False


	def set_translation(self, *args, **kwargs):
		super(BSplineTrajectory, self).set_translation(*args, **kwargs)
		self.is_computed = False
