#!/usr/bin/env python2

import rospy

from PubSubManager import *

from juggling_swarm_manager.srv import *
from crazyflie_trajectory_manager.srv import *
from std_srvs.srv import *

# utils
_traj_services = globals().keys()
_config_trajs = [name for name in _traj_services if name[-len('Request'):] == 'Request' and name[:-len('Request')] + 'Response' in _traj_services]

def service_response_class_from_request(request_obj):
	req_class_name = type(request_obj).__name__
	if req_class_name in _config_trajs:
		res_class_name = req_class_name[:-len('Request')] + 'Response'
		return globals()[res_class_name]
	else:
		return None

def service(func):
	def wrapper(req):
		res_class = service_response_class_from_request(req)
		if res_class is None:
			return func(req, None)
		else:
			res = res_class()
			return func(req, res)
	return wrapper

# The real business

class SwarmManager(object):
	def __init__(self, drones=['crazyflie_0']):
		self.manager = PubSubManager('swarm_manager', anonymous=False)

		self.drones = drones

		self.proxy_prefix = '/swarm/proxy/'
		self.to_proxy_list = [
			{'name': 'trajectory_manager/load_trajectory',
			'from_type': Proxy_LoadTrajectory, 'to_type': LoadTrajectory},
			{'name': 'trajectory_manager/start_trajectory',
			'from_type': Proxy_Empty, 'to_type': Empty},
			{'name': 'trajectory_manager/stop_trajectory',
			'from_type': Proxy_Empty, 'to_type': Empty},
			{'name': 'trajectory_manager/visualize',
			'from_type': Proxy_VisualizeTrajectory, 'to_type': VisualizeTrajectory},
			{'name': 'trajectory_manager/next_trajectory', 
			'from_type': Proxy_Empty, 'to_type': Empty},
			{'name': 'trajectory_manager/trajectory/config_bool',
			'from_type': Proxy_ConfigTrajectoryBool, 'to_type': ConfigTrajectoryBool},
			{'name': 'trajectory_manager/trajectory/config_float',
			'from_type': Proxy_ConfigTrajectoryFloat, 'to_type': ConfigTrajectoryFloat},
			{'name': 'trajectory_manager/trajectory/config_string',
			'from_type': Proxy_ConfigTrajectoryString, 'to_type': ConfigTrajectoryString},
			{'name': 'trajectory_manager/trajectory/config_vector',
			'from_type': Proxy_ConfigTrajectoryVector, 'to_type': ConfigTrajectoryVector},
			{'name': 'trajectory_manager/takeoff',
			'from_type': Proxy_Empty, 'to_type': Empty},
			{'name': 'trajectory_manager/land',
			'from_type': Proxy_Empty, 'to_type': Empty},
			{'name': 'trajectory_manager/translate_trajectory',
			'from_type': Proxy_TranslateTrajectory, 'to_type': TranslateTrajectory},
			{'name': 'trajectory_manager/set_trajectory_translation8',
			'from_type': Proxy_TranslateTrajectory, 'to_type': TranslateTrajectory}
			]

		self.proxies = {}

		self._build_proxies()

	def _build_proxies(self):
		for to_proxy in self.to_proxy_list:
			self.proxies[to_proxy['name']] = [None] * len(self.drones)
			for drone_idx, drone in enumerate(self.drones):
				try:
					serv_name = drone+'/'+to_proxy['name']
					self.manager.rospy.wait_for_service(serv_name, timeout=2)
					self.proxies[to_proxy['name']][drone_idx] = self.manager.add_client_service(serv_name, to_proxy['to_type'])

				except self.manager.rospy.ROSException:
					print 'ERROR WHILE LOADING SERVICE FROM DRONE', drone
			self.manager.add_server_service(self.proxy_prefix+to_proxy['name'],
				to_proxy['from_type'], self._create_proxy_callback(to_proxy))

	def _create_proxy_callback(self, proxy_object):
		@service
		def cb(req, res):
			if req.drone_id not in range(len(self.drones)):
				proxy_success = False
			else:
				proxied_req = proxy_object['to_type']._request_class()
				try:
					proxied_req.payload = req.payload
				except:
					pass

				proxy_service = self.proxies[proxy_object['name']][req.drone_id]
				if proxy_service is None:
					proxy_success = False
				else:
					proxied_res = proxy_service(proxied_req)
					try:
						result = proxied_res.result
						proxy_success = True
					except:
						proxy_success = False
						pass

			res.proxy_success = proxy_success
			try:
				res.result = result
			except:
				pass

			return res

		return cb

if __name__ == '__main__':
	drones = rospy.get_param('drone_prefixes', ['crazyflie_0'])
	swarm = SwarmManager(drones)
	rospy.spin()