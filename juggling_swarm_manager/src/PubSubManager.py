#!/usr/bin/env python

import rospy
import time

#returns ms since the epoch
def millis():
        return time.time() * 1000

class TopicException(Exception):
    pass

class PubSubManager():
	def __init__(self, node_name, anonymous=True):
		self.subscribers = {} # topic => rospy.Subscriber
		self.publishers = {} # topic => rospy.Publisher
		self.server_services = {} # name => rospy.Service
		self.client_services = {} # name => rospy.ServiceProxy

		self.states = {} # topic => {updated_at: ms_timestamp, data: value}
		self.callbacks = [] # {topics: [], last_call: ms_timestamp, func: callback, type: any/all}

		self.rospy = rospy

		rospy.init_node(node_name, anonymous=anonymous)


	def add_subscriber(self, topic, message_type, callback=None):

		def cb(msg):
			self.states[topic] = {'updated_at':millis(), 'data': msg}

			if callback:
				callback(self.states[topic])

			self.callback_handler(topic)


		self.states[topic] = {'updated_at': 0.0, 'data':None}
		self.subscribers[topic] = rospy.Subscriber(topic, message_type, cb)


	def add_publisher(self, topic, message_type, queue_size=10, **kwargs):
		self.publishers[topic] = rospy.Publisher(topic, message_type, queue_size=queue_size, **kwargs)


	def add_callback(self, topics, callback, cb_type='any'):
		topics_set = set(topics)
		if (len(topics_set - set(self.states)) > 0):
			raise TopicException('Cannot add a callback to a non-subscribed topic')

		self.callbacks.append({'topics':topics, 'last_call':millis(), 'func':callback, 'type':cb_type})


	def callback_handler(self, topic):

		to_call = [c for c in self.callbacks if topic in c['topics']]
		to_call_any = [c for c in to_call if c['type'] == 'any']
		to_call_all = [c for c in to_call if c['type'] == 'all']

		def all_ok(func):
			return len(
				# get only topics that have not been updated since last call
				filter(lambda t: self.states[t]['updated_at'] <= func['last_call'], func['topics'])
				) == 0

		to_call_all = [c for c in to_call_all if all_ok(c)]

		to_call = to_call_any + to_call_all
		for c in to_call:
			c['last_call'] = millis()
			c['func'](self)


	def publish(self, topic, message):
		if topic in self.publishers:
			return self.publishers[topic].publish(message)
		else:
			raise TopicException('Cannot publish to a topic where no publisher has been added')


	def spin(self):
		rospy.spin()


	def add_server_service(self, name, srv_file, callback):
		self.server_services[name] = rospy.Service(name, srv_file, callback)
		return self.server_services[name]

	def add_client_service(self, name, srv_file):
		self.client_services[name] = self.rospy.ServiceProxy(name, srv_file)
		return self.client_services[name]

	def call_service(self, name, *args, **kwargs):
		return self.client_services[name](*args, **kwargs)


if __name__ == '__main__':
	from std_msgs.msg import String as msgString

	manager = PubSubManager('test_sub', anonymous=False)

	def cb_factory(cb_name):
		def cb(msg):
			print cb_name, ':', msg
		return cb

	manager.add_subscriber('/hello', msgString, cb_factory('callback creating from subscribe'))
	manager.add_subscriber('/byebye', msgString)

	manager.add_callback(['/hello'], cb_factory('any callback on /hello'))
	manager.add_callback(['/hello', '/byebye'], cb_factory('all callback on /hello and /byebye'), cb_type='all')

	rospy.spin()
