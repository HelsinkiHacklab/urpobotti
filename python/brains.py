#!/usr/bin/env python

import zmq
import time

context = zmq.Context()

pinger = context.socket(zmq.SUB)
pinger.connect("tcp://urpobotti.local:7578")
pinger.setsockopt_string(zmq.SUBSCRIBE, u'')

imu = context.socket(zmq.SUB)
imu.connect("tcp://urpobotti.local:7574")
imu.setsockopt_string(zmq.SUBSCRIBE, u'')

motor = context.socket(zmq.DEALER)
motor.connect("tcp://urpobotti.local:7575")
encoder = context.socket(zmq.SUB)
encoder.connect("tcp://urpobotti.local:7576")
encoder.setsockopt_string(zmq.SUBSCRIBE, u'')

poller = zmq.Poller()
poller.register(pinger, zmq.POLLIN)
poller.register(imu, zmq.POLLIN)
poller.register(motor, zmq.POLLIN)
poller.register(encoder, zmq.POLLIN)

distances = [255, 255, 255] # right, center, left
attitude = {}
startup = time.time()

def send_motor(left, right):
	client.send_multipart(msg)

def stop_motors():
	motor.send_multipart(['setspeeds','0','0'])

import atexit
atexit.register(stop_motors)

prev = ""

def deadzone(value, dead=0.05): # default 5%
	if abs(value) - dead <= 0.0:
		return 0.0
	elif value < 0:
		return map(value, -1.0, -dead, -1.0, 0.0)
	elif value > 0:
		return map(value, dead, 1.0, 0.0, 1.0)

def map(value, physical_min, physical_max, virtual_min, virtual_max):
	return (value - physical_min) * (virtual_max - virtual_min) / (physical_max - physical_min) + virtual_min

def clamp(value, vmin, vmax):
	if value < vmin:
		return vmin
	elif value > vmax:
		return vmax
	else:
		return value

while True:
	socks = dict(poller.poll())

	if pinger in socks and socks[pinger] == zmq.POLLIN:
		# ['pingreport', '0', '54']
		msg = pinger.recv_multipart()

		if msg[0] == 'pingreport':
			distance = float(msg[2])
			if msg[2] == '0':
				distance == 255.0
			if distance > 50.0:
				distance = 255.0
			distances[int(msg[1])] = distance

	if imu in socks and socks[imu] == zmq.POLLIN:
		msg = imu.recv_multipart()
		attitude[msg[0]] = msg[1]

	if motor in socks and socks[motor] == zmq.POLLIN:
		msg = motor.recv_multipart()

	if encoder in socks and socks[encoder] == zmq.POLLIN:
		msg = encoder.recv_multipart()
		print "encoder", msg

	forward = clamp(map(min(distances), 10.0, 255.0, 0.0, 1.0)*6.0, 0.0, 1.0)

	if distances[1] < 10:
		left_x = 1.0
		forward = 0.0
	else:
		left_x = clamp((distances[0]/255.0 - distances[2]/255.0) * 6, -1.0, 1.0)

	left_motor = forward * 255.0 * (1.0-abs(left_x))
	right_motor = forward * 255.0 * (1.0-abs(left_x))
	left_motor += (left_x * 255.0) * abs(left_x)
	right_motor += (-left_x * 255.0) * abs(left_x)

	msg = ['setspeeds', str(int(right_motor)), str(int(left_motor))]
	
	if msg != prev: # lazy way to check if we should send an update =D
		prev = msg
		motor.send_multipart(msg)
#		print int(left_motor), int(right_motor), distances

	
#	print distances, attitude
