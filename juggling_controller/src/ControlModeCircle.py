from ControlMode import ControlMode
from circle_fit import fit_circle

# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

import numpy as np
from sklearn.decomposition import PCA

from scipy import optimize
import math


# n-dimensionnal distance computation
def distance(p0, p1):
	return math.sqrt(sum(
		[(p0[i] - p1[i])**2
		for i in 
		range(0,
			  min(len(p0), len(p1)))]))


# delete near points from a list 
def delete_close_points(points, precision=0.02, accumulate=False):
	prev = points[0]
	res = [points[0]]
	dist = 0
	for p in points[1:]:
		if accumulate:
			dist += distance(p, prev)
		else:
			dist = distance(p, prev)
		if dist >= precision:
			res.append(p)
			dist = 0
		prev = p
	if points[-1] not in res:
		res.append(points[-1])
	return res


def vec3_to_tuple(v):
	return (v.x, v.y, v.z)



class ControlModeCircle(ControlMode):
	def __init__(self, *args, **kwargs):
		super(ControlModeCircle, self).__init__(*args, **kwargs)
		self.recording = False
		self.points = []

	@staticmethod
	def get_name():
		return 'btimed'

	def on_input(self, glove, buttons_diff):
		if buttons_diff['select_button'] == 1:
			self.recording = True
		elif buttons_diff['select_button'] == -1:
			self.recording = False

			# viz
			self.points = delete_close_points(self.points)

			# fig = plt.figure(figsize=(7,5))
			# ax = Axes3D(fig)
			# line1 = ax.plot(*zip(*self.points))
			# ax.set_xlim(0,4)
			# ax.set_ylim(0,4)
			# ax.set_zlim(0,4)
			# plt.show()

			# pca
			pts = np.array(self.points)
			pca = PCA(n_components=2)
			transformed = pca.fit_transform(pts)

			pca_x = np.array(zip(*transformed)[0])
			pca_y = np.array(zip(*transformed)[1])

			# circle regression

			try:
				center_2b, R_2b = fit_circle(transformed, epsilon=0.01)
			except Exception as e:

				print 'Error while fitting a circle :', e
				# barycenter
				x_m = np.mean(pca_x)
				y_m = np.mean(pca_y)

				def calc_R(xc, yc):
					""" calculate the distance of each data points from the center (xc, yc) """
					return np.sqrt((pca_x-xc)**2 + (pca_y-yc)**2)

				def f_2b(c):
					""" calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
					Ri = calc_R(*c)
					return Ri - Ri.mean()

				center_estimate = x_m, y_m
				center_2b = optimize.least_squares(f_2b, center_estimate).x

				Ri_2b        = calc_R(*center_2b)
				R_2b         = Ri_2b.mean()

			xc_2b, yc_2b = center_2b

			# PCA + circle viz
			# plt.plot(pca_x, pca_y)
			# circle1=plt.Circle(center_2b,R_2b,color='r')
			# plt.gcf().gca().add_artist(circle1)

			# plt.axis([-1, 1,-1,1])

			# plt.show()


			# generate circle
			generated_points = []
			for angle in np.arange(0,2*math.pi,math.pi/100):
				generated_points.append([xc_2b + math.cos(angle) * R_2b, yc_2b + math.sin(angle) * R_2b])
			generated_points = np.array(delete_close_points(generated_points, accumulate=True, precision=0.1))

			# inverse PCA
			circle_points = pca.inverse_transform(generated_points)

			self.call_service('trajectory_manager/trajectory/config_bool',
				tid=1, key='loop', value=True)

			for p in circle_points:
				self.call_service('trajectory_manager/trajectory/config_vector',
					tid=1, key='add_point', value=p)

			self.call_service('trajectory_manager/visualize',
				tid=1)

			self.points = []
			self.send()


		if self.recording:
			self.points.append(vec3_to_tuple(glove.position))

