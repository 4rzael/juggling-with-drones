from Trajectory import Trajectory
from TrajectoryUtils import distance, delete_useless_points
import time
import numpy as np
from geometry_msgs.msg import Pose, Vector3

def millis():
	return int(time.time() * 1000.0)

# Public arguments :
# * orbiter_speed : The speed of the virtual orbiter in m/s: (x,y,z)
# * star_position : The position of the real "star": (x,y,z)
# * star_mass : The mass of the virtual star in kgm
# * distance_multiplier : The multiplier between real and virtual space distances : -Inf -> Inf
# * time_multiplier : The multiplier between real and virtual times : -Inf -> Inf
# * viz_seconds : Visualization. The time of the visualized trajectory : 0.0f -> Inf
# * viz_precision : Visualization. The precision of the visualized trajectory : 1 -> Inf
# * viz_position : Visualization. The initial position of the visualized trajectory : (x,y,z)

class GravitationTrajectory(Trajectory):
	def __init__(self, star_position=(0, 0, 0), star_mass=1000, orbiter_speed=(0,0,0),
		distance_multiplier=1, time_multiplier=1,
		viz_seconds=2, viz_precision=0.2, viz_position=(0,0,0),
		**kwargs):

		super(GravitationTrajectory, self).__init__(**kwargs)

		self.add_setter('orbiter_speed', self.set_orbiter_speed)
		self.add_setter('star_position', self.set_star_position)
		self.add_setter('star_mass', self.set_star_mass)
		self.add_setter('distance_multiplier', self.set_distance_multiplier)
		self.add_setter('time_multiplier', self.set_time_multiplier)

		self['orbiter_speed'] = orbiter_speed
		self['star_position'] = star_position
		self['star_mass'] = star_mass
		self['distance_multiplier'] = distance_multiplier
		self['time_multiplier'] = time_multiplier
		self['viz_seconds'] = viz_seconds
		self['viz_precision'] = viz_precision
		self['viz_position'] = viz_position

		self.first_call = True
		self.last_update_time = 0


	# for vizualisation purpose
	def visualize(self):
		speed = self['orbiter_speed']
		drone_position = self['viz_position']
		precision = self['viz_precision']
		seconds = self['viz_seconds']

		viz = []
		for i in range(0, seconds * 100):
			orbiter, drone = self._compute_one_tick(speed, drone_position, 0.01)

			speed = orbiter[1]
			drone_position = drone[2]
			viz.append(drone_position)

		viz = delete_useless_points(viz, precision=precision)
		return (viz, [self['viz_position']])


	# returns tuple(
	# 	tuple(orbiter_acceleration, orbiter_speed, orbiter_position),
	# 	tuple(drone_acceleration, drone_speed, drone_position),
	# )
	def _compute_one_tick(self, orbiter_speed, drone_pos, real_deltaTime):
		scaled_deltaTime = real_deltaTime * self['time_multiplier']
		drone_pos = np.array(drone_pos)
		orbiter_speed = np.array(orbiter_speed)

		# Convert to the simulated space, centered on the star and zoomed by distance_multiplier
		orbiter_position = (drone_pos - self['star_position']) * self['distance_multiplier']

		distance_meter = np.linalg.norm(orbiter_position)
		# compute the force direction vector (normalize)
		direction = -orbiter_position / distance_meter
		direction = np.round(direction, 5)

		# compute orbiter Acceleration (OA), Speed (OV) and Position (OP)
		G = 6.674e-11
		OA = (G * direction * self['star_mass']) / ((distance_meter) ** 2)
		OV = orbiter_speed + OA * scaled_deltaTime
		OP = orbiter_position + OV * scaled_deltaTime

		# convert back the the real space-time
		# acceleration is active acceleration, thus we remove the earth gravity
		DA = (OA / self['distance_multiplier']) * self['time_multiplier'] - np.array([0, 0, -9.81])
		DV = (OV / self['distance_multiplier']) * self['time_multiplier']
		DP = (OP / self['distance_multiplier'] + self['star_position'])

		return (
			(tuple(OA), tuple(OV), tuple(OP)),
			(tuple(DA), tuple(DV), tuple(DP))
		)


	# Returns a pose
	def get_next_pose(self, drone_position):
		print '==================================='
		print 'GET NEXT POSE FROM', drone_position
		if not self.started: # while not started : Don't move
			pose = Pose()
			pose.position = Vector3(*drone_position)
			return pose

		if self.first_call:
			self.first_call = False
			self.last_update_time = millis()

		deltaTime = (millis() - self.last_update_time) / 1000.0

		orbiter, drone = self._compute_one_tick(self['orbiter_speed'], drone_position, deltaTime)

		self['orbiter_speed'] = orbiter[1]
		self.last_update_time = millis()

		pose = Pose()
		pose.position = Vector3(*drone[2])
		print 'NEXT POSE =', pose.position
		print '==================================='
		return pose


	def set_orbiter_speed(self, orbiter_speed):
		self._set_values(orbiter_speed=np.array(orbiter_speed))

	def set_star_position(self, star_position):
		if star_position is not None:
			self._set_values(star_position=np.array(star_position))
		else:
			self._set_values(star_position=None)

	def set_star_mass(self, star_mass):
		if star_mass is not None:
			self._set_values(star_mass=float(star_mass))
		else:
			self._set_values(star_mass=None)

	def set_distance_multiplier(self, distance_multiplier):
		self._set_values(distance_multiplier=float(distance_multiplier))

	def set_time_multiplier(self, time_multiplier):
		self._set_values(time_multiplier=float(time_multiplier))

	def start(self):
		super(GravitationTrajectory, self).start()
		self.first_call = True
