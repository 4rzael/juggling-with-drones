#!/usr/bin/env python2

from sys import argv

from TrajectoryManager import TrajectoryManager

from BSplineTrajectory import BSplineTrajectory
from BSplineTimedTrajectory import BSplineTimedTrajectory
from GravitationTrajectory import GravitationTrajectory
from BallLikeTrajectory import BallLikeTrajectory
from HoverTrajectory import HoverTrajectory

# circle
bspline_points = [
	(1.5, 1.2, 2),
	(2.3, 2, 2),
	(1.5, 2.8, 2),
	(0.7, 2, 2)
]

if __name__ == '__main__':
	chosen = None
	if len(argv) >= 2:
		chosen = argv[1]

	tm = TrajectoryManager()
	tm.bbox.set_z(minimum=0.2, maximum=4)
	tm.bbox.set_x(minimum=0.2, maximum=2.5)
	tm.bbox.set_y(minimum=0.2, maximum=3.5)
	tm.visualize_bbox()

	trajectories = {
		'hover': HoverTrajectory(
			position=(2,2,2)
		),
		'bspline': BSplineTrajectory(
			precision=0.1,
			key_points=bspline_points,
			loop=True
		),
		'bspline_time': BSplineTimedTrajectory(
			2000,
			precision=0.1,
			key_points=bspline_points,
			loop=True
		),
		'ball': BallLikeTrajectory(
			drone_speed=(2, 0, 4),
			time_multiplier=0.5,
			viz_position=(1, 1, 2)
		),
		'grav': GravitationTrajectory(
			star_position=(1.5, 2, 1),
			star_mass=5.972e24,
			orbiter_speed=(7350, 0, 0),
			distance_multiplier=7378*1000,
			time_multiplier=1000,
			viz_position=(1.5, 1.25, 1)
		)
	}

	if chosen is not None:
		try:
			tm.load_trajectory(trajectories.get(chosen, None))
			tm.visualize_trajectory()
		except:
			pass
	tm.start()
	tm.manager.spin()
