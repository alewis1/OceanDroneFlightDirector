from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time
import math

podCoordList = [] #What coordinates to drop pods at?
ind = 0
vehicle = None
homeCoords = None #Needs to be set

def start():
	global vehicle
	"""Start the vehicle, and takeoff to a height of 1 meter"""
	vehicle = connect("/dev/ttyACM0")

	while not vehicle.is_armable:
		print "Waiting on Vehicle..."
		time.sleep(1)
	
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
		
	while not vehicle.armed:
		print "Arming Vehicle..."
		time.sleep(1)
	
	vehicle.simple_takeoff(1)
	time.sleep(5)

def disarm():
	"""Disarms the vehicle. The vehicle automatically enters a "return to home" mode upon being disarmed, and performs a safe automatic touchdown"""
	vehicle.armed = False
  
def returnHome():
	"""Returns the vehicle to present home coords"""
	setLoc(homeCoords)
            


def kill():
	"""Deconstruct the vehicle object"""
	vehicle.close()
	print "Vehicle Closed"

def setVel(x, y, z, t):
	"""Move vehicle in direction (x,y,z) for time t"""
	print "Moving vehicle in vector (" + str(x) + ", " + str(y) + ", " + str(z) + ") for duration " + str(t)
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_LOCAL_NED,
		0b0000111111000111,
		0, 0, 0,
		x, y, z,
		0, 0, 0,
		0, 0)
	for x in range(t):
		vehicle.send_mavlink(msg)
		time.sleep(1)

def setLoc(coords):
	print "Moving vehicle to location " + str(coords)
	currentLoc = vehicle.location.global_relative_frame
	targetLoc = LocationGlobal(coords[0], coords[1], coords[2])
	dist = get_distance_meters(currentLoc, targetLoc)
	vehicle.simple_goto(targetLoc)
	while dist > 5:
		currentLoc = vehicle.location.global_relative_frame
		dist = get_distance_meters(currentLoc, targetLoc)

def release():
	"""Release a pod."""
	pass

def findPod():
	"""Move to the pod's last known position, find it, and pick it up"""
	try:
		coords = podCoordList[ind-1]
	except:
		coords = podCoordList[0]
	setLoc(coords)
	setLoc(useLoRa())
	pickUpPod()

def useLoRa():
	"""Use LoRa to find the actual position of the pod"""
	return None #GPS coords of pod

def pickUpPod():
	"""Align with pod, and use Unspecified Mechanism to pick it up"""
	pass

def pyramidTest():
	"""Starts the drone, moves in a 1x1x0.5 meter pyramid centered on the orgin, and returns home"""
	start()
	setVel(0,0,-1,1)
	setVel(0,0,0,1)
	setVel(0.5,0.5,0,1)
	setVel(0,0,0,1)
	setVel(-0.5,0,0,2)
	setVel(0,0,0,1)
	setVel(0,-0.5,0,2)
	setVel(0,0,0,1)
	setVel(0.5,0,0,2)
	setVel(0,0,0,1)
	setVel(0,0.5,0,2)
	setVel(0,0,0,1)
	setVel(-0.5,-0.5,-0.5,1)
	setVel(0,0,0.25,6)
	disarm()

def vertTest():
	"""Starts the drone, moves up a meter, and returns home"""
	start()
	setVel(0,0,-1,2)
	setVel(0,0,0.5,4)
	disarm()

def dropPod():
	"""Go to the next location to drop a pod, and drop a pod"""
	try:
		coords = podCoordList[ind]
	except:
		coords = podCoordList[0]
		ind = 0
	setLoc(coords)
	ind += 1
	release()

def locTest():
	start()
	coords = (0,0,0) #Enter coords before testing
	setLoc(coords)
	disarm()

def main():
        pyramidTest()
        kill()

main()
