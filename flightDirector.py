import drone.py

def drop():
    drone.start()
    drone.setVel(0,0,-2,10) #Ascend to safe height
    drone.dropPod()
    drone.returnHome()
    drone.disarm()
    drone.kill()

def retrieve():
    drone.start()
    drone.setVel(0,0,-2,10)
    drone.findPod()
    drone.returnHome()
    drone.disarm()
    drone.kill()
    
