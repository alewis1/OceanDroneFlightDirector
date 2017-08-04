from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time
import math

podCoords = None 
vehicle = None
homeCoords = None 

def start():
	global vehicle
	"""Start the vehicle, and takeoff to a height of 1 meter"""
	vehicle = connect("/dev/ttyACM0") #This address was obtained with trial and error. It may change.

	while not vehicle.is_armable: #Wait until the drone is done with all pre-flight conditions (Gyro Calibration, GPS Lock, etc)
		print "Waiting on Vehicle..."
		time.sleep(1)
	
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	vehicle.groundspeed = 5
		
	while not vehicle.armed: #Wait until the vehicle actually arms. Check the Safety Button if you're getting caught on this.
		print "Arming Vehicle..."
		time.sleep(1)
	
	vehicle.simple_takeoff(5) #Built-in method. Takes off to a height of 5 meters. 
	time.sleep(10) #Wait until the drone is finished taking off. This time is not exact, and could probably be tuned to be much more accurate.

def disarm():
	"""Disarms the vehicle. The vehicle automatically enters a "return to home" mode upon being disarmed, and performs a safe automatic touchdown"""
	vehicle.armed = False

def get_distance_meters(locA, locB):
        """Gets the distance between two location objects, in meters"""
	dlat = locB.lat - locA.lat
	dlon = locB.lon - locA.lon
	return math.sqrt((dlat**2) + (dlon**2)) * 1.1131195e5

def returnHome():
	"""Returns the vehicle to present home coords"""
	setLoc(homeCoords)
	    
def get_loc(dNorth, dEast):
	"""Returns a location offset by a certain amount from the current position. Inaccurate over very long distances or near the poles."""
	currentLoc = vehicle.location.global_frame
	rad = 6378137.0
	dLat = dNorth/rad
	dLon = dEast/(rad*math.cos(math.pi*currentLoc.lat/180))
	return LocationGlobal(currentLoc.lat+(dLat*180/math.pi), currentLoc.lon+(dLon*180/math.pi), currentLoc.alt)

def kill():
	"""Deconstruct the vehicle object"""
	vehicle.close()
	print "Vehicle Closed"

def setVel(x, y, z, t):
	"""Move vehicle in direction (x,y,z) for time t"""
	print "Moving vehicle in vector (" + str(x) + ", " + str(y) + ", " + str(z) + ") for duration " + str(t)
	msg = vehicle.message_factory.set_position_target_local_ned_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111, 0, 0, 0, x, y, z, 0, 0, 0, 0, 0) #Generates a command to set velocity to a vector of (x,y,z)
	for x in range(t): #Send the command every second until duration runs out
		vehicle.send_mavlink(msg)
		time.sleep(1)

def setLoc(coords):
        """Moves vehicle to position, represented by a tuple of (Latitude, Longitude, Altitude)"""
	print "Moving vehicle to location " + str(coords)
	currentLoc = vehicle.location.global_frame
	targetLoc = LocationGlobal(coords[0], coords[1], coords[2]) #Generates a LocationGlobal object from coords
	dist = get_distance_meters(currentLoc, targetLoc)
	vehicle.simple_goto(targetLoc) #Built-in command, tells vehicle to move to a given LocationGlobal object
	while dist > 5: #Wait until vehicle actually reaches given location
		currentLoc = vehicle.location.global_relative_frame
		dist = get_distance_meters(currentLoc, targetLoc)
		print dist
		print vehicle.mode
		time.sleep(1)
	print "Got there!"

def setLocRel(dNorth, dEast):
	"""Move to a position a certain amount away from the current position"""
	pos = get_loc(dNorth, dEast)
	lat, lon, alt = pos.lat, pos.lon, pos.alt
	setLoc((lat, lon, alt))

def release():
	"""Release a pod."""
	pass #This method not implemented yet

def setHome():
	global homeCoords
	"""Sets the home location to the current position"""
	pos = vehicle.location.global_frame
	lat, lon, alt = pos.lat, pos.lon, pos.alt
	homeCoords = (lat, lon, alt)

def findPod():
	"""Move to the pod's last known position, find it, and pick it up"""
	pass #This method not implemented yet

def useLoRa():
	"""Use LoRa to find the actual position of the pod"""
	return None #This method not implemented yet

def pickUpPod():
	"""Align with pod, and use Unspecified Mechanism to pick it up"""
	pass #This method not implemented yet

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
	pass #This method not implemented yet

def waypoint(rLat, rLon, rAlt=0, delay=0):
	"""Generates a WAYPOINT mavlink command"""
	pos = get_loc(rLat, rLon)
	lat, lon = pos.lat, pos.lon
	return Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, delay, 0, 0, 0, lat, lon, rAlt)

def rtl():
	"""Generates a RTL mavlink command"""
	return Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def locTest():
        """Traces a square, using GUIDED mode. Be sure that GCS failsafe is off before attempting this"""
	start()
	print "1"
	setHome()
	print "2"
	print homeCoords
	setLocRel(20, 20)
	print 3
	setLocRel(-40, 0)
	print 4
	setLocRel(0, -40)
	print 5
	setLocRel(40, 0)
	print 6
	setLocRel(0, 40)
	returnHome()
	disarm()

def locTestAuto():
        """Traces a square (and does a curtsy at each corner) using AUTO mode. Untested"""
	start()
	cmds = vehicle.commands()
	cmds.clear()
	cmds.add(waypoint(20,20,0,0))
	cmds.add(waypoint(0,0,-1,0))
	cmds.add(waypoint(0,0,1,1))
	cmds.add(waypoint(-40,0,0,0))
	cmds.add(waypoint(0,0,-1,0))
	cmds.add(waypoint(0,0,1,1))
	cmds.add(waypoint(0,-40,0,0))
	cmds.add(waypoint(0,0,-1,0))
	cmds.add(waypoint(0,0,1,1))
	cmds.add(waypoint(40,0,0,0))
	cmds.add(waypoint(0,0,-1,0))
	cmds.add(waypoint(0,0,1,1))
	cmds.add(waypoint(0,40,0,0))
	cmds.add(waypoint(0,0,-1,0))
	cmds.add(waypoint(0,0,1,1))
	cmds.add(rtl())
	cmds.add(waypoint(0,0,0,0)) #dummy waypoint to detect End Of Mission
	cmds.upload()
	vehicle.mode = VehicleMode("AUTO")
	while vehicle.commands.next < vehicle.commands.count:
		time.sleep(1)
	vehicle.mode = VehicleMode("GUIDED")
	disarm()

def main():
	locTest()
	kill()

main()
