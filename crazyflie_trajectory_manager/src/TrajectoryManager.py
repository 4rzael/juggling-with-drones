from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA
import std_msgs.msg
from std_srvs.srv import *
from visualization_msgs.msg import *

from PubSubManager import *

from Trajectory import Trajectory
from TrajectoryUtils import TrajectoryBBOX, service, FLIGHT_STATES, tuple3_add

from crazyflie_trajectory_manager.srv import *

from BSplineTrajectory import BSplineTrajectory
from BSplineTimedTrajectory import BSplineTimedTrajectory
from GravitationTrajectory import GravitationTrajectory
from BallLikeTrajectory import BallLikeTrajectory
from HoverTrajectory import HoverTrajectory
from LandTrajectory import LandTrajectory
from Juggle3DTrajectory import Juggle3DTrajectory


def vec3_to_tuple(v):
	return (v.x, v.y, v.z)

class TrajectoryManager(object):
	def __init__(self, bbox=TrajectoryBBOX()):
		self.listening = False
		self.manager = PubSubManager('trajectory_manager', anonymous=False)
		self.manager.add_publisher('goal', PoseStamped)
		self.manager.add_publisher('trajectory/viz/line', Marker, latch=True)
		self.manager.add_publisher('trajectory/viz/points', Marker, latch=True)
		self.manager.add_publisher('trajectory/viz/bbox', Marker, latch=True)

		try:
			self.manager.add_server_service('trajectory_manager/load_trajectory',
				LoadTrajectory, self.srv_load_trajectory)
			self.manager.add_server_service('trajectory_manager/start_trajectory',
				Empty, self.srv_start_trajectory)
			self.manager.add_server_service('trajectory_manager/stop_trajectory',
				Empty, self.srv_stop_trajectory)
			self.manager.add_server_service('trajectory_manager/visualize',
				VisualizeTrajectory, self.srv_visualize)
			self.manager.add_server_service('trajectory_manager/next_trajectory',
				Empty, self.srv_next_trajectory)

			self.manager.add_server_service('trajectory_manager/trajectory/config_bool',
				ConfigTrajectoryBool, self.srv_config_trajectory)
			self.manager.add_server_service('trajectory_manager/trajectory/config_float',
				ConfigTrajectoryFloat, self.srv_config_trajectory)
			self.manager.add_server_service('trajectory_manager/trajectory/config_string',
				ConfigTrajectoryString, self.srv_config_trajectory)
			self.manager.add_server_service('trajectory_manager/trajectory/config_vector',
				ConfigTrajectoryVector, self.srv_config_trajectory)

			self.manager.add_server_service('trajectory_manager/takeoff',
				Empty, self.srv_takeoff)
			self.manager.add_server_service('trajectory_manager/land',
				Empty, self.srv_land)

			self.manager.add_server_service('trajectory_manager/translate_trajectory',
				TranslateTrajectory, self.srv_translate_trajectory)
			self.manager.add_server_service('trajectory_manager/set_trajectory_translation',
				TranslateTrajectory, self.srv_set_trajectory_translation)

			self.manager.add_server_service('ready', Empty, self.srv_nop)


		except Exception as e:
			print 'ERROR WHILE CREATING SERVICES :', e

		self.manager.add_client_service('takeoff', Empty)
		self.manager.add_client_service('land', Empty)

		self.bbox = bbox
		self.trajectories = [None, None]
		self.drone_position = None

		self.trajectory_translation = (0,0,0)

		# ENUM
		self.flight_state = FLIGHT_STATES.LANDED

	def visualize_trajectory(self, tid=0):
		vline, vpoints = self.trajectories[tid].visualize()

		line = Marker()
		line.scale = Vector3(0.1, 0.1, 0.1)
		line.color = ColorRGBA(1, 0, 1, 1)
		line.header = std_msgs.msg.Header()
		line.header.stamp = self.manager.rospy.Time.now()
		line.header.frame_id = '/world'
		line.type = 4
		line.id = 0xF00D
		line.lifetime = self.manager.rospy.Duration(0)
		# bbox cropping
		line.points = [Point(*p) for p in self.bbox.crop_trajectory(vline)]
		line.colors = [ColorRGBA(1, 0, float(idx) / len(line.points), 1) for idx, p in enumerate(line.points)]
		self.manager.publish('trajectory/viz/line', line)

		pts = Marker()
		pts.scale = Vector3(0.1, 0.1, 0.1)
		pts.color = ColorRGBA(1, 1, 0, 1)
		pts.header = std_msgs.msg.Header()
		pts.header.stamp = self.manager.rospy.Time.now()
		pts.header.frame_id = '/world'
		pts.type = 8
		pts.id = 0xF00D + 1
		pts.lifetime = self.manager.rospy.Duration(0)
		pts.points = [Point(*p) for p in vpoints]
		pts.colors = [ColorRGBA(1, float(idx) / len(pts.points), 0, 1) for idx, p in enumerate(pts.points)]
		self.manager.publish('trajectory/viz/points', pts)

	def visualize_bbox(self):
		bbox = Marker()
		bbox.header = std_msgs.msg.Header()
		bbox.header.stamp = self.manager.rospy.Time.now()
		bbox.header.frame_id = '/world'
		bbox.type=1
		bbox.color = ColorRGBA(1, 0, 0, 0.2)

		def mean((a, b)):
			return float(a + b) / 2

		bbox.pose.position = Vector3(*map(mean, (self.bbox.x, self.bbox.y, self.bbox.z)))
		bbox.scale = Vector3(*map(lambda (a,b): min([100, b-a]), (self.bbox.x, self.bbox.y, self.bbox.z)))
		self.manager.publish('trajectory/viz/bbox', bbox)


	def load_trajectory(self, traj, tid=0):
		assert issubclass(traj.__class__, Trajectory) == True
		self.trajectories[tid] = traj


	def start(self):
		if self.listening == False:
			self.manager.add_subscriber('pose', PoseStamped, self._on_pose)
			self.listening = True


	def _on_pose(self, poseWrapper):
		self.drone_position = vec3_to_tuple(poseWrapper['data'].pose.position)

		if self.trajectories[0] and self.trajectories[0].started:
			pose = poseWrapper['data']
			ps = PoseStamped()
			ps.header = std_msgs.msg.Header()
			ps.header.stamp = self.manager.rospy.Time.now()
			ps.header.frame_id = '/world'
			ps.pose = self.trajectories[0].get_next_pose(self.drone_position)

			# bbox cropping
			ps.pose.position = Vector3(*self.bbox.crop_point(vec3_to_tuple(ps.pose.position)))
			self.manager.publish('goal', ps)

	# Services controllers

	def trajectory_start(self):
		if self.trajectories[0]:
			self.trajectories[0].start()

	def trajectory_stop(self):
		if self.trajectories[0]:
			self.trajectories[0].stop()

	def trajectory_config(self, tid, **kwargs):
		if self.trajectories[tid]:
			for k in kwargs:
				self.trajectories[tid][k] = kwargs[k]

	def takeoff(self):
		if self.flight_state == FLIGHT_STATES.LANDED:

			# if there is nothing to do for the drone, make it hover at 1.5 meters from the ground
			if self.trajectories[0] is None and self.drone_position is not None:
				self.load_trajectory(HoverTrajectory(manager=self), tid=0)
				pos = (self.drone_position[0], self.drone_position[1], 1.5)
				self.trajectory_config(tid=0, position=pos)
				self.trajectory_start()

			self.manager.call_service('takeoff')
			self.flight_state = FLIGHT_STATES.FLYING

	def land(self):
		if self.flight_state == FLIGHT_STATES.FLYING:
			self.manager.call_service('land')
			self.flight_state = FLIGHT_STATES.LANDED

	def next_trajectory(self):
		self.trajectories = self.trajectories[1:] + [None]
		if self.trajectories[0] is None:
			self.translate_trajectory([-p for p in self.trajectory_translation])

	def translate_trajectory(self, translation):
		self.trajectory_translation = tuple3_add(self.trajectory_translation, translation)

		for t in self.trajectories:
			if t is not None:
				t.set_translation(self.trajectory_translation)
		return self.trajectory_translation

	# Services views

	@service
	def srv_load_trajectory(self, req, res):
		print 'LOADING TRAJECTORY', req.payload.tid, req.payload.trajectory_type

		TRAJS = {
			'hover': HoverTrajectory,
			'bspline': BSplineTrajectory,
			'btimed': BSplineTimedTrajectory,
			'ball': BallLikeTrajectory,
			'grav': GravitationTrajectory,
			'land_at_pos': LandTrajectory,
			'juggle3D': Juggle3DTrajectory
		}

		if req.payload.trajectory_type in TRAJS:
			t = TRAJS[req.payload.trajectory_type](manager=self)
			print t.items()
			self.load_trajectory(t, req.payload.tid)
			res.result = True
		else:
			res.result = False

		return res

	@service
	def srv_start_trajectory(self, req, res):
		print 'STARTING TRAJECTORY'
		self.trajectory_start()
		return res

	@service
	def srv_stop_trajectory(self, req, res):
		print 'STOPPING TRAJECTORY'
		self.trajectory_stop()
		return res

	@service
	def srv_config_trajectory(self, req, res):
		print 'CONFIGURING TRAJECTORY', req.payload.tid, req.payload.key, '=', req.payload.value

		if self.trajectories[req.payload.tid] is not None:
			self.trajectory_config(req.payload.tid, **{req.payload.key: req.payload.value})
			res.result = True
		else:
			res.result = False

		return res

	@service
	def srv_visualize(self, req, res):
		if self.trajectories[req.payload.tid] is not None:
			self.visualize_trajectory(req.payload.tid)
			res.result = True
		else:
			res.result = False

		self.visualize_bbox()
		return res

	@service
	def srv_takeoff(self, req, res):
		self.takeoff()
		return res

	@service
	def srv_land(self, req, res):
		self.land()
		return res

	@service
	def srv_next_trajectory(self, req, res):
		self.next_trajectory()
		return res

	@service
	def srv_translate_trajectory(self, req, res):
		t = self.translate_trajectory(req.payload.translation)
		res.result = t
		return res

	@service
	def srv_set_trajectory_translation(self, req, res):
		to_translate = (
			req.payload.translation[0] - self.trajectory_translation[0],
			req.payload.translation[1] - self.trajectory_translation[1],
			req.payload.translation[2] - self.trajectory_translation[2]
		)

		t = self.translate_trajectory(to_translate)
		res.result = t
		return res

	@service
	def srv_nop(self, req, res):
		return res
