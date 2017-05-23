import math
from geometry_msgs.msg import Pose, Vector3

def tuple3_add(t1, t2):
	return (
		t1[0] + t2[0],
		t1[1] + t2[1],
		t1[2] + t2[2]
		)

# n-dimensionnal distance computation
def distance(p0, p1):
	return math.sqrt(sum(
		[(p0[i] - p1[i])**2
		for i in 
		range(0,
			  min(len(p0), len(p1)))]))

def compute_trajectory_distance(traj, loop=False):
	dist = 0
	prev = traj[0] if len(traj) > 0 else None
	for p in traj:
		dist += distance(p, prev)
		prev = p
	if loop and len(traj) > 0:
		dist += distance(traj[-1], traj[0])
	return dist

# delete near points from a list 
def delete_useless_points(points, precision=0.1):
	dist = 0
	prev = points[0]
	res = [points[0]]
	for p in points[1:]:
		dist += distance(p, prev)
		if dist >= precision:
			res.append(p)
			dist = 0
		prev = p
	if points[-1] not in res:
		res.append(points[-1])
	return res

class TrajectoryBBOX(object):
	def __init__(self,
				 x=[float('-inf'), float('inf')],
				 y=[float('-inf'), float('inf')],
				 z=[float('-inf'), float('inf')]):
		self.x = x
		self.y = y
		self.z = z

	def set_value(self, value, minimum=None, maximum=None):
		if minimum is not None:
			value[0] = minimum
		if maximum is not None:
			value[1] = maximum
		return value

	def set_x(self, minimum=None, maximum=None):
		return self.set_value(self.x, minimum, maximum)

	def set_y(self, minimum=None, maximum=None):
		return self.set_value(self.y, minimum, maximum)

	def set_z(self, minimum=None, maximum=None):
		return self.set_value(self.z, minimum, maximum)

	def is_inside(self, point):
		def _in((p, arr)):
			return arr[0] <= p and p <= arr[1]

		return reduce(lambda a,b: a and b,
			map(_in,
				zip(p, [self.x, self.y, self.z])
				)
			)

	def crop_point(self, point):
		def _crop_one((p, arr)):
			return min(max(arr[0], p), arr[1])

		return map(_crop_one,
			zip(point, [self.x, self.y, self.z])
			)

	def crop_trajectory(self, points):
		try:
			return map(self.crop_point, points)
		except:
			return []


# Given a Service request, returns the corresponding response type
try:
	from std_srvs.srv import *
	from crazyflie_trajectory_manager.srv import *
	_traj_services = globals().keys()

	_config_trajs = [name for name in _traj_services if name[-len('Request'):] == 'Request' and name[:-len('Request')] + 'Response' in _traj_services]
except Exception as e:
	_config_trajs = []

print _config_trajs

def service_response_class_from_request(request_obj):
	req_class_name = type(request_obj).__name__
	if req_class_name in _config_trajs:
		res_class_name = req_class_name[:-len('Request')] + 'Response'
		return globals()[res_class_name]
	else:
		return None

def service(func):
	def wrapper(self, req):
		res_class = service_response_class_from_request(req)
		if res_class is None:
			return func(self, req, None)
		else:
			res = res_class()
			return func(self, req, res)
	return wrapper

# ENUMS

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

FLIGHT_STATES = enum('LANDED', 'FLYING')

def returns_pose(method):
	def wrapper(self, *args, **kwargs):
		P = Pose()
		P.position = Vector3(*method(self, *args, **kwargs))
		return P
	return wrapper