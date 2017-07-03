from dronekit import connect, VehicleMode, LocationGlobal

vehicle = connect("/dev/ttyACM0")
vehicle.armed = False
vehicle.close()
