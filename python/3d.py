import string
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from numpy import array
import sys,math,time
import random
ESCAPE = '\033'
SPACE = ' '
window = 0
rx,ry = 0.0, 0.0

pos = 0
paused = False

import zmq
context = zmq.Context()
pinger = context.socket(zmq.SUB)
pinger.connect("tcp://urpobotti.local:7578")
pinger.setsockopt_string(zmq.SUBSCRIBE, u'')
encoder = context.socket(zmq.SUB)
encoder.connect("tcp://urpobotti.local:7576")
encoder.setsockopt_string(zmq.SUBSCRIBE, u'')
poller = zmq.Poller()
poller.register(pinger, zmq.POLLIN)
poller.register(encoder, zmq.POLLIN)

distances = [255, 255, 255] # right, center, left
tires = [0.0, 0.0]

def time_diff_iter():
	prev = time.time()
	while True:
		now = time.time()
		yield(now - prev)
		prev = now

time_diff = time_diff_iter()

def InitGL(Width, Height):				
	glClearColor(0.0, 0.0, 0.0, 0.0)	
	glClearDepth(1.0)					
	glDepthFunc(GL_LESS)				
	glEnable(GL_DEPTH_TEST)				
	glShadeModel(GL_SMOOTH)				
	glEnable (GL_BLEND)
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
	glFogi(GL_FOG_MODE, GL_LINEAR)
	glFogfv(GL_FOG_COLOR, [0.0, 0.0, 0.0, 1.0])
	glFogf(GL_FOG_DENSITY, 0.35)
	glHint(GL_FOG_HINT,    GL_DONT_CARE)
	glFogf(GL_FOG_START,   1.0)
	glFogf(GL_FOG_END,     10.0)
	glEnable(GL_FOG)

	glEnable(GL_COLOR_MATERIAL)

	glEnable(GL_LIGHTING)
	glEnable(GL_LIGHT0)
	glLightfv(GL_LIGHT0, GL_POSITION, (0.0,0.0,0.0,1.0))
	glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0,1.0,1.0,1.0))

	glPointSize(1.0)
	glLineWidth(1.1)
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()					
	gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)
	glMatrixMode(GL_MODELVIEW)

def ReSizeGLScene(Width, Height):
	if Height == 0:
		Height = 1
	glViewport(0, 0, Width, Height)
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)

def DrawGLScene():
	global rx,ry,pos,paused
	if not paused:
		ry += time_diff.next() * 45.0 
	if ry>360.0: ry-=360.0
	rx=0.0

	socks = dict(poller.poll())
	if pinger in socks and socks[pinger] == zmq.POLLIN:
		# ['pingreport', '0', '54']
		msg = pinger.recv_multipart()

		if msg[0] == 'pingreport':
			distance = int(msg[2])
			distances[int(msg[1])] = distance

	if encoder in socks and socks[encoder] == zmq.POLLIN:
		msg = encoder.recv_multipart()

		if msg[0] == 'ppsreport':
			right, left = int(msg[1]), int(msg[2])
			tires[0] += right
			tires[1] += left
			print tires


	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
	glMatrixMode(GL_MODELVIEW)
	glLoadIdentity()
#	glTranslatef(0.0, 0.0, -4.0)
#	glRotatef(ry, 0.0, 1.0, 0.0)
#	glRotatef(rx, 1.0, 0.0, 0.0)
	glColor4f(1.0, 1.0, 1.0, 1.0)
	gluLookAt(
		0.0, 3.0, -2.0,
		0.0, 0.0, 1.0,
		0.0, 1.0, 0.0,
		)

	glPushMatrix()
	glRotatef(90.0, 1.0, 0.0, 0.0)
	glutSolidTorus( 0.25, 0.5, 25, 25 )
	glPopMatrix()

	glPushMatrix()
	# sensor base
	glTranslatef(0.0, 0.0, 0.5)
	glColor4f(1.0, 1.0, 0.0, 1.0)
	glutSolidCube( 0.5 )

# left
	if(distances[2] != 0):
		glColor4f(1.0, 0.0, 0.0, 1.0)
		glPushMatrix()
		glRotatef(45.0, 0.0, 1.0, 0.0)
		glTranslatef(0.0, 0.0, distances[2] / 10.0)
		glutSolidCube( 0.5 )
		glPopMatrix()

# front
	if(distances[1] != 0):
		glColor4f(0.0, 1.0, 0.0, 1.0)
		glPushMatrix()
		glTranslatef(0.0, 0.0, distances[1] / 10.0)
		glutSolidCube( 0.5 )
		glPopMatrix()

# right
	if(distances[0] != 0):
		glColor4f(0.0, 0.0, 1.0, 1.0)
		glPushMatrix()
		glRotatef(-45.0, 0.0, 1.0, 0.0)
		glTranslatef(0.0, 0.0, distances[0] / 10.0)
		glutSolidCube( 0.5 )
		glPopMatrix()
	glPopMatrix()

# left tire
	glColor4f(0.5, 0.5, 0.5, 1.0)
	glPushMatrix()
	glTranslatef(0.75, 0.0, 0.0)
	glRotatef(90.0, 0.0, 1.0, 0.0)
	glRotatef(tires[1]*3.6, 0.0, 0.0, 1.0)
#	glutSolidTorus(0.1, 0.25, 4, 4)
	glutSolidCube(0.5)
	glPopMatrix()

# right tire
	glColor4f(0.5, 0.5, 0.5, 1.0)
	glPushMatrix()
	glTranslatef(-0.75, 0.0, 0.0)
	glRotatef(90.0, 0.0, 1.0, 0.0)
	glRotatef(tires[0]*3.6, 0.0, 0.0, 1.0)
#	glutSolidTorus(0.1, 0.25, 4, 4)
	glutSolidCube(0.5)
	glPopMatrix()

	glutSwapBuffers()

def keyPressed(*args):
	global window,paused
	if args[0] == ESCAPE:
		sys.exit()
	if args[0] == SPACE:
		paused = not paused

def main():
	global window
	glutInit(sys.argv)
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
	glutInitWindowSize(800, 600)
	glutInitWindowPosition(0, 0)
	window = glutCreateWindow("Dist's awesome robot visualizer!")
	glutDisplayFunc(DrawGLScene)
	glutIdleFunc(DrawGLScene)
	glutReshapeFunc(ReSizeGLScene)
	glutKeyboardFunc(keyPressed)
	InitGL(800, 600)
	glutMainLoop()

print "Hit ESC key to quit."

main()
