#
# Weather update client
# Connects SUB socket to tcp://localhost:5556
# Collects weather updates and finds avg temp in zipcode
#

import sys
import zmq

# Open socket for listening to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print "Connected to axis data"
socket.connect ("tcp://localhost:7574")

# Subscribe to all the relevant axis
socket.setsockopt(zmq.SUBSCRIBE, "roll")
socket.setsockopt(zmq.SUBSCRIBE, "pitch")
socket.setsockopt(zmq.SUBSCRIBE, "yaw")

while True:
	strings = socket.recv_multipart()
	print(strings[1])
