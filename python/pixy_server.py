#!/usr/bin/env python
# -*- coding: utf-8 -*-
#from pixy import *
#from ctypes import *
import ctypes
import pixy

# Pixy Python SWIG get blocks example #

print ("Pixy Python SWIG Example -- Get Blocks")

# Initialize Pixy Interpreter thread #
pixy.pixy_init()

class Blocks (ctypes.Structure):
  _fields_ = [ ("type", ctypes.c_uint),
               ("signature", ctypes.c_uint),
               ("x", ctypes.c_uint),
               ("y", ctypes.c_uint),
               ("width", ctypes.c_uint),
               ("height", ctypes.c_uint),
               ("angle", ctypes.c_uint) ]

blocks = pixy.Block()

print("Going loopy")

# Wait for blocks #
while 1:

  count = pixy.pixy_get_blocks(1, blocks)

  if count > 0:
    # Blocks found #
    print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d ANGLE=%3d]' % (blocks.type, blocks.signature, blocks.x, blocks.y, blocks.width, blocks.height, blocks.angle)
