#!/usr/bin/env python

import hid # tested with hidapi==0.7.99-5
import zmq
import time

# 1: left stick, x, 0=left, 255=right
# 2: left stick, y, 0=up, 255=down
# 3: right stick, x, 0=left, 255=right
# 4: right stick, y, 0=up, 255=down
# 8: left trigger, 0=not pressed, 255=fully pressed
# 9: right trigger, 0=not pressed, 255=fully pressed
# 20: x-rot
# 24: z-rot

devices = [(x['vendor_id'],x['product_id']) for x in hid.enumerate() if x['product_string'] == u'Wireless Controller']

assert len(devices)==1, "joystick count != 1"

d = hid.device()
d.open(*devices[0])

context = zmq.Context()
client = context.socket(zmq.DEALER)
client.connect("tcp://urpobotti.local:7575")

def map(value, physical_min, physical_max, virtual_min, virtual_max):
	return (value - physical_min) * (virtual_max - virtual_min) / (physical_max - physical_min) + virtual_min

def deadzone(value, dead=0.05): # default 5%
	if abs(value) - dead <= 0.0:
		return 0.0
	elif value < 0:
		return map(value, -1.0, -dead, -1.0, 0.0)
	elif value > 0:
		return map(value, dead, 1.0, 0.0, 1.0)

mode = 0

prev = ""
while True:
	values = tuple(d.read(25)) # 25 bytes from PS4 controller
	
	print(' '.join("%d:%%2x"%x for x in range(25)) % values)
	left_x = deadzone(map(values[1], 0.0, 255.0, -1.0, 1.0))
	left_y = deadzone(map(values[2], 0.0, 255.0, -1.0, 1.0))
	right_x = deadzone(map(values[3], 0.0, 255.0, -1.0, 1.0))
	right_y = deadzone(map(values[4], 0.0, 255.0, -1.0, 1.0))
	left_trigger = map(values[8], 0.0, 255.0, 0.0, 1.0)
	right_trigger = map(values[9], 0.0, 255.0, 0.0, 1.0)

	if values[5] & 0x10 == 0x10: # square switches to tank controls
		mode = 0
	elif values[5] & 0x20 == 0x20: # X switches to car controls
		mode = 1

	if mode == 0: # Tank controls
		left_motor = left_y * -255.0
		right_motor = right_y * -255.0
	elif mode == 1: # Car-ish controls (still allows turning without moving forward)
		forward = right_trigger - left_trigger
		left_motor = forward * 255.0 * (1.0-abs(left_x))
		right_motor = forward * 255.0 * (1.0-abs(left_x))
		left_motor += (left_x * 255.0) * abs(left_x)
		right_motor += (-left_x * 255.0) * abs(left_x)
		# print ("%.2f "*7) % (left_x, left_y, right_x, right_y, left_trigger, right_trigger, forward)
	
	msg = ['setspeeds', str(int(right_motor)), str(int(left_motor))]
	
	if msg != prev: # lazy way to check if we should send an update =D
		prev = msg

		client.send_multipart(msg)
		client.recv()

		#print(str(int(right_motor)), str(int(left_motor)))
