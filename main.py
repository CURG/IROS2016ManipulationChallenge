
import argparse
import random
import pkg_resources
pkg_resources.require("klampt>=0.6.2")

#Klampt v0.7.x
from klampt import *
from klampt import vis 
from klampt.vis.glrobotprogram import *
from klampt.math import *
from klampt.model import collide
from klampt.io import resource
from klampt.sim import *

from moving_base_control import *
import importlib
import os
import time
import sys
import math
import tf_conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3

delta = 0.001

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

def xyzrpy_to_xform(xyzrpy):

	r = tf_conversions.Rotation.RPY(*xyzrpy[3:])
	pose = Pose(Point(*xyzrpy[:3]), Quaternion(*r.GetQuaternion()))
	frame = tf_conversions.fromMsg(pose)
	transform = tf_conversions.toMatrix(frame)
	rotMatrix = transform[0:3,0:3].flatten().tolist()
	tran = [pose.position.x, pose.position.y, pose.position.z]

	return rotMatrix, tran



def test_grasp(robotname,object_set,objectname, xyzrpy):


	world = WorldModel()
	world.loadElement("data/terrains/plane.env")
	robot = make_moving_base_robot(robotname,world)
	m_object = make_object(object_set,objectname,world)
	
	#this sets the initial condition for the simulation
	xform = xyzrpy_to_xform(xyzrpy)
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
	
	#this code manually updates the visualization
	vis.add("world",world)
	vis.show()
	t0 = time.time()
	count = 0
	initial_z = m_object.getTransform()[1][2]
	while vis.shown():
		vis.lock()
		sim.simulate(0.01)
		sim.updateWorld()
		vis.unlock()
		t1 = time.time()
		time.sleep(max(0.01-(t1-t0),0.001))
		t0 = t1
		count +=1
		if count == 800:
			break

		if m_object.getTransform()[1][2] > initial_z+0.05:
			print "grasp successful"
			vis.kill()
			return True
	print "grasp failed"
	vis.kill()

	return False


if __name__ == '__main__':

	parser = argparse.ArgumentParser(description='Picking up object with Klampt')
	parser.add_argument('--dataset_name', type=str,
		default='apc2015')

	parser.add_argument('--robot_name', type=str,
		default='reflex_col')

	parser.add_argument('--object_name', type=str,
		default='cheezit_big_original')

	parser.add_argument('xyzrpy', type=float, nargs='+',
		default=[1,2,3,4,5,6])


	args = parser.parse_args()
	
	#just plan grasping
	test_grasp(args.robot_name, args.dataset_name, args.object_name, args.xyzrpy)