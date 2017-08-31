
import argparse
import random
import pkg_resources
pkg_resources.require("klampt>=0.6.2")
if pkg_resources.get_distribution("klampt").version >= '0.7':
	#Klampt v0.7.x
	from klampt import *
	from klampt import vis 
	from klampt.vis.glrobotprogram import *
	from klampt.math import *
	from klampt.model import collide
	from klampt.io import resource
	from klampt.sim import *
else:
	#Klampt v0.6.x
	from klampt import *
	from klampt import visualization as vis
	from klampt import resource
	from klampt import robotcollide as collide
	from klampt.simulation import *
	from klampt.glrobotprogram import *

from moving_base_control import *
import importlib
import os
import time
import sys
import math

delta = 0.001

box_dims = (0.5,0.5,0.3)
shelf_dims = (0.4,0.4,0.3)
shelf_offset = 0.6
shelf_height = 0.7
moving_base_template_fn = 'data/robots/moving_base_template.rob'
object_template_fn = 'data/objects/object_template.obj'
objects = {}
#objects['ycb'] = [f for f in sorted(os.listdir('data/objects/ycb'))]
objects['apc2015'] = [f for f in sorted(os.listdir('data/objects/apc2015'))]
robots = ['reflex_col']

object_geom_file_patterns = {
	'ycb':['data/objects/ycb/%s/meshes/tsdf_mesh.stl','data/objects/ycb/%s/meshes/poisson_mesh.stl'],
	'apc2015':['data/objects/apc2015/%s/textured_meshes/optimized_tsdf_textured_mesh.ply']
}
#default mass for objects whose masses are not specified, in kg
default_object_mass = 0.5
object_masses = {
	'ycb':dict(),
	'apc2015':dict(),
}
robot_files = {
	'reflex_col':'data/robots/reflex_col.rob'
}


def mkdir_p(path):
	"""Quietly makes the directories in path"""
	import os, errno
	try:
		os.makedirs(path)
	except OSError as exc: # Python >2.5
		if exc.errno == errno.EEXIST and os.path.isdir(path):
			pass
		else: raise

def make_object(object_set,objectname,world):
	"""Adds an object to the world using its geometry / mass properties
	and places it in a default location (x,y)=(0,0) and resting on plane."""
	for pattern in object_geom_file_patterns[object_set]:
		objfile = pattern%(objectname,)
		objmass = object_masses[object_set].get('mass',default_object_mass)
		f = open(object_template_fn,'r')
		pattern = ''.join(f.readlines())
		f.close()
		f2 = open("temp.obj",'w')
		f2.write(pattern % (objfile,objmass))
		f2.close()
		nobjs = world.numRigidObjects()
		if world.loadElement('temp.obj') < 0 :
			continue
		assert nobjs < world.numRigidObjects(),"Hmm... the object didn't load, but loadElement didn't return -1?"
		obj = world.rigidObject(world.numRigidObjects()-1)
		obj.setTransform(*se3.identity())
		bmin,bmax = obj.geometry().getBB()
		T = obj.getTransform()
		spacing = 0.006
		T = (T[0],vectorops.add(T[1],(-(bmin[0]+bmax[0])*0.5,-(bmin[1]+bmax[1])*0.5,-bmin[2]+spacing)))
		obj.setTransform(*T)
		obj.appearance().setColor(0.2,0.5,0.7,1.0)
		obj.setName(objectname)
		return obj
	raise RuntimeError("Unable to load object name %s from set %s"%(objectname,object_set))

def make_box(world,width,depth,height,wall_thickness=0.005,mass=float('inf')):
	"""Makes a new axis-aligned box centered at the origin with
	dimensions width x depth x height. Walls have thickness wall_thickness. 
	If mass=inf, then the box is a Terrain, otherwise it's a RigidObject
	with automatically determined inertia.
	"""
	left = Geometry3D()
	right = Geometry3D()
	front = Geometry3D()
	back = Geometry3D()
	bottom = Geometry3D()
	left.loadFile("data/objects/cube.tri")
	right.loadFile("data/objects/cube.tri")
	front.loadFile("data/objects/cube.tri")
	back.loadFile("data/objects/cube.tri")
	bottom.loadFile("data/objects/cube.tri")
	left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
	right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
	front.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,-depth*0.5,0])
	back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
	bottom.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,0])
	#bottom.setAABB([-width*0.5,-depth*0.5,0],[width*0.5,depth*0.5,wall_thickness])
	#left.setAABB([-width*0.5,-depth*0.5,0],[-width*0.5+wall_thickness,depth*0.5,height])
	#right.setAABB([width*0.5-wall_thickness,-depth*0.5,0],[width*0.5,depth*0.5,height])
	#front.setAABB([-width*0.5,-depth*0.5,0],[width*0.5,-depth*0.5+wall_thickness,height])
	#back.setAABB([-width*0.5,depth*0.5-wall_thickness,0],[width*0.5,depth*0.5,height])
	boxgeom = Geometry3D()
	boxgeom.setGroup()
	for i,elem in enumerate([left,right,front,back,bottom]):
		g = Geometry3D(elem)
		boxgeom.setElement(i,g)
	if mass != float('inf'):
		print "Making a box a rigid object"
		bmass = Mass()
		bmass.setMass(mass)
		bmass.setCom([0,0,height*0.3])
		bmass.setInertia([width/12,depth/12,height/12])
		box = world.makeRigidObject("box")
		box.geometry().set(boxgeom)
		box.appearance().setColor(0.6,0.3,0.2,1.0)
		box.setMass(bmass)
		return box
	else:
		print "Making a box a terrain"
		box = world.makeTerrain("box")
		box.geometry().set(boxgeom)
		box.appearance().setColor(0.6,0.3,0.2,1.0)
		return box

def make_shelf(world,width,depth,height,wall_thickness=0.005):
	"""Makes a new axis-aligned "shelf" centered at the origin with
	dimensions width x depth x height. Walls have thickness wall_thickness. 
	If mass=inf, then the box is a Terrain, otherwise it's a RigidObject
	with automatically determined inertia.
	"""
	left = Geometry3D()
	right = Geometry3D()
	back = Geometry3D()
	bottom = Geometry3D()
	top = Geometry3D()
	left.loadFile("data/objects/cube.tri")
	right.loadFile("data/objects/cube.tri")
	back.loadFile("data/objects/cube.tri")
	bottom.loadFile("data/objects/cube.tri")
	top.loadFile("data/objects/cube.tri")
	left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
	right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
	back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
	bottom.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,0])
	top.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,height-wall_thickness])
	shelfgeom = Geometry3D()
	shelfgeom.setGroup()
	for i,elem in enumerate([left,right,back,bottom,top]):
		g = Geometry3D(elem)
		shelfgeom.setElement(i,g)
	shelf = world.makeTerrain("shelf")
	shelf.geometry().set(shelfgeom)
	shelf.appearance().setColor(0.2,0.6,0.3,1.0)
	return shelf

def make_moving_base_robot(robotname,world):
	"""Converts the given fixed-base robot into a moving base robot
	and loads it into the given world.
	"""
	f = open(moving_base_template_fn,'r')
	pattern = ''.join(f.readlines())
	f.close()
	f2 = open("temp.rob",'w')
	f2.write(pattern 
		% (robot_files[robotname],robotname))
	f2.close()
	world.loadElement("temp.rob")
	return world.robot(world.numRobots()-1)



def launch_simple(robotname,object_set,objectname,use_box=False):
	"""Launches a very simple program that simulates a robot grasping an object from one of the
	databases. It first allows a user to position the robot's free-floating base in a GUI. 
	Then, it sets up a simulation with those initial conditions, and launches a visualization.
	The controller closes the hand, and then lifts the hand upward.  The output of the robot's
	tactile sensors are printed to the console.

	If use_box is True, then the test object is placed inside a box.
	"""
	world = WorldModel()
	world.loadElement("data/terrains/plane.env")
	robot = make_moving_base_robot(robotname,world)
	m_object = make_object(object_set,objectname,world)
	if use_box:
		box = make_box(world,*box_dims)
		m_object.setTransform(*se3.mul((so3.identity(),[0,0,0.01]),m_object.getTransform()))
	doedit = False
	xform = resource.get("%s/default_initial_%s.xform"%(object_set,robotname),description="Initial hand transform",default=robot.link(5).getTransform(),world=world)
	set_moving_base_xform(robot,xform[0],xform[1])
	xform = resource.get("%s/initial_%s_%s.xform"%(object_set,robotname,objectname),description="Initial hand transform",default=robot.link(5).getTransform(),world=world,doedit=False)
	if xform:
		set_moving_base_xform(robot,xform[0],xform[1])
	xform = resource.get("%s/initial_%s_%s.xform"%(object_set,robotname,objectname),description="Initial hand transform",default=robot.link(5).getTransform(),world=world,doedit=doedit)
	if not xform:
		print "User quit the program"
		return
	#this sets the initial condition for the simulation
	set_moving_base_xform(robot,xform[0],xform[1])

	#now the simulation is launched
	program = GLSimulationPlugin(world)
	sim = program.sim

	#setup some simulation parameters
	visPreshrink = True   #turn this to true if you want to see the "shrunken" models used for collision detection
	for l in range(robot.numLinks()):
		sim.body(robot.link(l)).setCollisionPreshrink(visPreshrink)
	for l in range(world.numRigidObjects()):
		sim.body(world.rigidObject(l)).setCollisionPreshrink(visPreshrink)

	#create a hand emulator from the given robot name
	module = importlib.import_module('plugins.'+robotname)
	#emulator takes the robot index (0), start link index (6), and start driver index (6)
	hand = module.HandEmulator(sim,0,6,6)
	sim.addEmulator(0,hand)

	#the result of simple_controller.make() is now attached to control the robot
	import simple_controller
	sim.setController(robot,simple_controller.make(sim,hand,delta))

	#the next line latches the current configuration in the PID controller...
	sim.controller(0).setPIDCommand(robot.getConfig(),robot.getVelocity())
	
	#this code uses the GLSimulationProgram structure, which gives a little more control over the visualization
	"""
	program.simulate = True
	vis.setPlugin(program)
	vis.show()
	while vis.shown():
		time.sleep(0.1)
	return
	"""
	
	#this code manually updates the visualization
	vis.add("world",world)
	vis.show()
	t0 = time.time()
	while vis.shown():
		vis.lock()
		sim.simulate(0.01)
		sim.updateWorld()
		vis.unlock()
		t1 = time.time()
		time.sleep(max(0.01-(t1-t0),0.001))
		t0 = t1
	return


if __name__ == '__main__':

	parser = argparse.ArgumentParser(description='Picking up object with Klampt')
	parser.add_argument('--dataset', type=str,
		default='apc2015')

	parser.add_argument('--robot', type=str,
		default='reflex_col')

	parser.add_argument('--objname', type=str,
		default='cheezit_big_original')

	args = parser.parse_args()

	dataset = args.dataset
	robot = args.robot
	objname = args.objname

	#just plan grasping
	launch_simple(robot,dataset,objname)
	vis.kill()
