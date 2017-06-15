from BallLikeTrajectory import BallLikeTrajectory
from TrajectoryUtils import delete_useless_points, returns_pose, millis, normalize_nparray

import numpy as np

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Public arguments :
# See BallLikeTrajectory
# base_x_vector : The x axis of the new coordinate system : (x,y,z)
# hand_vector : A vector in the x_z plan of the new coordinate system
class Juggle3DTrajectory(BallLikeTrajectory):
	def __init__(self, **kwargs):
		super(Juggle3DTrajectory, self).__init__(**kwargs)

		self['base_x_vector'] = None
		self['hand_vector'] = None
		self.add_setter('base_x_vector', self._set_base_x_vector)
		self.add_setter('hand_vector', self._set_hand_vector)
		self.add_setter('gravity', self._set_gravity)

	def _set_hand_vector(self, vec):
		self._set_values(hand_vector=normalize_nparray(np.array(vec)))
		self._compute_transform_matrix()

	def _set_base_x_vector(self, vec):
		self._set_values(base_x_vector=normalize_nparray(np.array(vec)))
		self._compute_transform_matrix()


	def _compute_transform_matrix(self):
		if self['base_x_vector'] is None or self['hand_vector'] is None:
			return
		# Y axis
		x_z_plan_normal = normalize_nparray(np.cross(self['hand_vector'], self['base_x_vector']))
		# Z axis
		x_y_plan_normal = normalize_nparray(np.cross(self['base_x_vector'], x_z_plan_normal))

		self.matrix_unrotate = np.matrix([
			self['base_x_vector'],
			x_z_plan_normal,
			x_y_plan_normal]
			).transpose()

		self.matrix_rotate = np.linalg.inv(self.matrix_unrotate)

		print 'TO ROTATE MATRIX'
		print self.matrix_rotate

		# Side effects ! o/
		self['gravity'] = self['gravity']


	def _set_gravity(self, grav):
		self._set_values(gravity=tuple((-self.matrix_unrotate.transpose()[2].A1) * np.linalg.norm(grav)))
		print 'GRAVITY SET', self['gravity']

	def _should_stop(self, point, init_pos):
		init_pos = np.array(init_pos)

		init_pos_rotated = self.matrix_rotate * np.matrix(init_pos).transpose()
		point_rotated = self.matrix_rotate * np.matrix(point).transpose()

		print 'INIT POS :', init_pos
		print 'POINT', point
		print 'SHOULD STOP :', init_pos_rotated.A1[2], point_rotated.A1[2]
		return point_rotated.A1[2] < init_pos_rotated.A1[2]

	def visualize(self, **kwargs):
		self._compute_transform_matrix()

		res = list(super(Juggle3DTrajectory, self).visualize(**kwargs))
		res[1] = [(1,1,1), tuple(self.matrix_unrotate.transpose()[0].A1 + np.array([1,1,1])),
				 (1,1,1), tuple(self.matrix_unrotate.transpose()[1].A1 + np.array([1,1,1])),
				 (1,1,1), tuple(self.matrix_unrotate.transpose()[2].A1 + np.array([1,1,1]))]
		return tuple(res)

