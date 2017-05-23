from Trajectory import Trajectory
from geometry_msgs.msg import Pose, Vector3
from TrajectoryUtils import tuple3_add

# Public arguments :
# * position : The position the drone should hover at : (x, y, z)

class HoverTrajectory(Trajectory):
	def __init__(self, position=(0,0,0), **kwargs):
		super(HoverTrajectory, self).__init__(**kwargs)
		self['position'] = position

	def get_next_pose(self, drone_position):
		pose = Pose()

		pose.position = Vector3(*tuple3_add(self['position'], self.translation))
		return pose

	def visualize(self, **kwargs):
		return ([], [tuple3_add(self['position'], self.translation)])
