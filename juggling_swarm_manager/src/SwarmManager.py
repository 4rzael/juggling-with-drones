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

def service(self=False):
	def service_decorator(func):
		if self is False:
			def wrapper(req):
				res_class = service_response_class_from_request(req)
				if res_class is None:
					return func(req, None)
				else:
					res = res_class()
					return func(req, res)
		else:
			def wrapper(_self, req):
				res_class = service_response_class_from_request(req)
				if res_class is None:
					return func(_self, req, None)
				else:
					res = res_class()
					return func(_self, req, res)
		return wrapper
	return service_decorator

# The real business

class SwarmManager(object):
	def __init__(self):
		self.manager = PubSubManager('swarm_manager', anonymous=False)

		self.nb_drones = int(self.manager.rospy.get_param('~nb_drones', default='2'))
		self.drones = self.manager.rospy.get_param('~drone_prefixes')[:self.nb_drones]
		self.active_drones = []
		bases = self.manager.rospy.get_param('~bases', [])
		self.bases = [None] * len(self.drones)
		try:
			for i,b in enumerate(bases):
				self.bases[i] = b
		except:
			print 'Cannot read bases'

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
			{'name': 'trajectory_manager/set_trajectory_translation',
			'from_type': Proxy_TranslateTrajectory, 'to_type': TranslateTrajectory},
			{'name': 'emergency',
			'from_type': Proxy_Empty, 'to_type': Empty}
			]

		self.proxies = {}

		self._get_active_drones()
		self._build_proxies()

		print self.proxies

		# self.manager.add_server_service('land_at_base', Proxy_Empty, self.srv_land_at_base)

	def _get_active_drones(self):
		self.active_drones = []
		for idx, d in enumerate(self.drones):
			try:
				self.manager.rospy.wait_for_service(d+'/ready', timeout=10)
				self.active_drones.append(idx)
			except:
				pass

	def _add_client_service(self, s_name, s_type):
		srvs = [None] * len(self.drones)
		for drone_idx in self.active_drones:
			drone = self.drones[drone_idx]

			serv_name = drone+'/'+s_name
			try:
				srvs[drone_idx] = self.manager.add_client_service(serv_name, s_type)

			except self.manager.rospy.ROSException:
				print 'ERROR WHILE LOADING SERVICE', serv_name
		return srvs

	def _build_proxies(self):
		for to_proxy in self.to_proxy_list:
			self.proxies[to_proxy['name']] = self._add_client_service(to_proxy['name'], to_proxy['to_type'])
			self.manager.add_server_service(self.proxy_prefix+to_proxy['name'],
				to_proxy['from_type'], self._create_proxy_callback(to_proxy))

	def _create_proxy_callback(self, proxy_object):
		@service()
		def cb(req, res):
			print 'PROXYING', proxy_object['to_type'], 'TO DRONE', req.drone_id

			if req.drone_id not in self.active_drones:
				print 'PROXY ERROR : DRONE NOT FOUND'
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
					print 'PROXY ERROR : PROXY NOT FOUND'
				else:
					proxied_res = proxy_service(proxied_req)
					try:
						result = proxied_res.result
						proxy_success = True
					except Exception as e:
						proxy_success = False
						print 'PROXY ERROR :', e
						pass

			res.proxy_success = proxy_success
			try:
				res.result = result
			except:
				pass

			print 'PROXY SUCCESS :', res.proxy_success
			return res

		return cb

	# @service(self=True)
	# def srv_land_at_base(self, req, res):
	# 	if req.drone_id not in range(len(self.drones)):
	# 		print 'ERROR : DRONE NOT FOUND'
	# 	else:
			

if __name__ == '__main__':
	swarm = SwarmManager()
	rospy.spin()