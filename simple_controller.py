from klampt import *
from klampt.math import vectorops,so3,se3
from moving_base_control import *

def make(sim,hand,dt):
	"""The make() function returns a 1-argument function that takes a SimRobotController and performs whatever
	processing is needed when it is invoked."""

	#get references to the robot's sensors (not properly functioning in Klamp't 0.6.x)
	f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
	contact_sensors = f1_proximal_takktile_sensors + f1_distal_takktile_sensors + f2_proximal_takktile_sensors + f2_distal_takktile_sensors + f3_proximal_takktile_sensors + f3_distal_takktile_sensors
	sim.updateWorld()
	xform = get_moving_base_xform(sim.controller(0).model())

	def controlfunc(controller):
		"""Place your code here... for a more sophisticated controller you could also create a class where the control loop goes in the __call__ method."""
		#print the contact sensors... you can safely take this out if you don't want to use it
		try:
			f1_contact = [s.getMeasurements()[0] for s in f1_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f1_distal_takktile_sensors]
			f2_contact = [s.getMeasurements()[0] for s in f2_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f2_distal_takktile_sensors]
			f3_contact = [s.getMeasurements()[0] for s in f3_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f3_distal_takktile_sensors]
			print "Contact sensors"
                        print "sim.getTime(): "  + str(sim.getTime())
			print "  finger 1:",[int(v) for v in f1_contact]
			print "  finger 2:",[int(v) for v in f2_contact]
			print "  finger 3:",[int(v) for v in f3_contact]
		except:
			pass
		t_lift = 6
		lift_traj_duration = 0.5
		if sim.getTime() < 0.05:
			#the controller sends a command to the hand: f1,f2,f3,preshape
			hand.setCommand([0.2,0.2,0.2,0])
		if sim.getTime() > t_lift:
			#the controller sends a command to the base after 1 s to lift the object
                        print "We are lifting!!!!"
			t_traj = min(1, max(0, (sim.getTime() - t_lift) / lift_traj_duration))
                        print "t_traj: " + str(t_traj)
			desired = se3.mul((so3.identity(), [0, 0, 0.10 * t_traj]), xform)
                        print "desired[0]: " + str(desired[0])
                        print "desired[1]: " + str(desired[1])
			send_moving_base_xform_PID(controller, desired[0], desired[1])
		#need to manually call the hand emulator
                print type(hand)

		hand.process({},dt)
	return controlfunc
