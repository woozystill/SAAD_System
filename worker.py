import time
from multiprocessing import Manager, Process
from web_server import run_server
import serial

import gpiozero
from time import sleep
import numpy as np
import sys
import gps
import collections
import serial
import math

from magnetometerLibrary import QMC5883LCompass
from escControl import ArduinoESC

def cleanupGPS():
    try:
        session.close()
    except Exception:
        pass
        
def send_command(gps_serial, command):
    """
    Send a command to the GPS module over serial.
    Args:
        gps_serial: Serial connection to the GPS module.
        command: Command string to send (NMEA command).
    """
    gps_serial.write(command.encode('ascii'))
    sleep(0.1)  # Give the GPS module time to process the command

def setGPSPollingRate():
    gps_serial = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=0.1)
    print("Connected to GPS module.")

    # Command to set the update rate to 10 Hz (100 ms)
    update_rate_command = "$PMTK220,1000*2F\r\n"

    # Command to set the position fix interval to 100 ms
    fix_rate_command = "$PMTK300,1000,0,0,0,0*2C\r\n"

    print("Setting polling rate to 10 Hz...")
    send_command(gps_serial, update_rate_command)
    send_command(gps_serial, fix_rate_command)

    print("Polling rate maximized to 10 Hz. Verifying...")
    while True: 
        # Read GPS output to confirm new rate
        data = gps_serial.readline().decode('ascii', errors='ignore')
        if data:
            print(data.strip())

def readGPS(session):

    # Fetch the next report from gpsd
    report = session.next()

    # Check if the report contains a position fix
    if report['class'] == 'TPV':
        # Latitude and longitude may be missing if there's no fix
        latitude = getattr(report, 'lat', 'N/A')
        longitude = getattr(report, 'lon', 'N/A')
        altitude = getattr(report, 'alt', 'N/A')
        speed = getattr(report, 'speed', 'N/A')
        if(isinstance(latitude, str) or isinstance(longitude, str) or isinstance(altitude, str) or isinstance(speed, str)):
            return (0, 0, 0)


        return (1, latitude, longitude)
    return (0, 0, 0)

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing between two points.
    Arguments:
        lat1, lon1: Latitude and Longitude of the first point (in decimal degrees).
        lat2, lon2: Latitude and Longitude of the second point (in decimal degrees).
    Returns:
        Bearing in degrees (0 = North, 90 = East, etc.).
    """
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    delta_lon = lon2 - lon1

    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    
    initial_bearing = math.atan2(x, y)
    
    return initial_bearing
    
def calculate_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    R = 6371e3
    a = math.sin(delta_lat / 2.0) * math.sin(delta_lat / 2.0) + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2.0) * math.sin(delta_lon / 2.0)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c
    
def getSmoothAzimuth(sensor):
    total = 0
    samples = 1
    for i in range(samples):
        sensor.read()
        total = total + (math.radians(sensor.getAzimuth()) / samples)
        sleep(0.01)
    return total

def getAzimuth(sensor):
    sensor.read()
    return sensor.getAzimuth()

def control_code(target_lat, target_long, curr_lat, curr_long, heading):
    # PARAMETERS:
    full_speed_dist = 20 #meters
    min_dist = 5
    full_speed_theta = math.radians(90)
    max_speed = 50
    min_speed = 20
    
    thetaToTarget = calculate_bearing(target_lat, target_long, curr_lat, curr_long)
    thetaDiff = thetaToTarget - heading
    
    distance = calculate_distance(target_lat, target_long, curr_lat, curr_long)
    
    if(distance < min_dist):
        return (0.01, 0.01)
    
    # Scale forward power down linearly after it gets within "full_speed_dist"
    forward_power = 0
    if(distance > full_speed_dist):
        forward_power = max_speed
    else:
        forward_power = max_speed - max_speed * (1.0 - (distance / full_speed_dist))
        
    if(abs(forward_power) < min_speed):
        forward_power = abs(forward_power) / forward_power * min_speed
    
    rotation_power = 0;
    thetaDiff = (thetaDiff + 3.0 * np.pi) % (2.0 * np.pi) - np.pi
    print("Angle: " + str(thetaDiff))
    print("Distance: " + str(distance))


    if(abs(thetaDiff) > full_speed_theta):
        rotation_power = max_speed
    else:
        rotation_power = max_speed - max_speed * (1.0 - (thetaDiff / full_speed_theta))
        
    if(abs(forward_power) < min_speed):
        forward_power = abs(forward_power) / forward_power * min_speed
        
    if(abs(thetaDiff) > np.pi / 2.0):
        thetaDiff = (abs(thetaDiff) / thetaDiff) * np.pi / 2.0
        
    rotation_fraction = abs(math.sin(thetaDiff))
    
    left_thrust = (1.0 - abs(rotation_fraction)) * forward_power + (-rotation_fraction) * rotation_power
    right_thrust = (1.0 - abs(rotation_fraction)) * forward_power + (rotation_fraction) * rotation_power
    
    print("Forward power: " + str(forward_power) + ", Rotational power:" + str(rotation_power) + "SIN: " + str(rotation_fraction))

    print("Thrusts: " + str(left_thrust) + ", " + str(right_thrust))
    
    return (left_thrust, right_thrust)

def degrees_to_compass(degrees):
    directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    
    index = round(degrees / 22.5) % 16  # 360/16 = 22.5 degrees per direction
    return directions[index]

def main(shared_data):
    # Connect to the gpsd daemon
    session = gps.gps(mode=gps.WATCH_ENABLE)
    # Pin configuration
    #LEFT_MOTOR = 12
    #RIGHT_MOTOR = 13  # GPIO pin connected to the PWM device
    
    # Clock (PWM frequency in Hz)
    #CLOCK = 50
    
    ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)

    rightMotor = ArduinoESC(ser, 'b')
    leftMotor = ArduinoESC(ser, 'a')
    leftGroundMotor = ArduinoESC(ser, 'y')
    rightGroundMotor = ArduinoESC(ser, 'z')
    
    
    #right_motor_control = ZMREsc(pin=LEFT_MOTOR)
    #left_motor_control = ZMREsc(pin=RIGHT_MOTOR)

    # Setup a circular buffer
    pastGPSPositions = collections.deque(maxlen=5)
    pastGPSPositions.append((32.231311667, -110.957048333))
    pastGPSPositions.append((32.231322461931185, -110.95715212191602))
    pastGPSPositions.append((32.23132366727646, -110.95729089480041))
    pastGPSPositions.append((32.23132864818837, -110.9574352940373))
    pastGPSPositions.append((32.231332086967264, -110.95781008036023))
    
    shared_data["target_lats"].append(32.2822648)
    shared_data["target_longs"].append(-111.0340554)

    # Initialize sensor
    sensor = QMC5883LCompass()
    sensor.init()
    #sensor.calibrate()
    #sleep(100)
        
    sensor.setCalibration(0, 1633.0, 
        -361.0, 993.0, 
        0, 916.0)


    # calibrateThrusters()
    rightMotor.sendCommand(0.0)
    leftMotor.sendCommand(0.0)
    rightGroundMotor.sendCommand(0.0)
    leftGroundMotor.sendCommand(0.0)
    sleep(3)
    print("Thrusters initialized.")

    prevPWM = (0.0, 0.0)
    last_left = 0.0
    last_right = 0.0
    last_ground_left = 0.0
    last_ground_right = 0.0
    counter = 0
    lastLight = False
    while True:
        shared_data["batteryLevel"] = str(55 + 0 * int(leftMotor.reqVoltage())) + "%"
        shared_data["heading"] = 91 + 0 * int(leftMotor.reqMag())
        print(shared_data["heading"])
        shared_data["heading_string"] = degrees_to_compass(shared_data["heading"]) + " " + str(shared_data["heading"])
        sleep(0.05)  # Adjust sleep time as needed
        #print("COUNTER: " + str(counter))
        counter = counter + 1
        if(counter == 1):
            print("READING GPS!!")
            success, lat, lon = readGPS(session)
            counter = 0
            if(success):
                print("SUCCESS!!" + str(lat) + " " + str(lon))
                shared_data["currLat"] = lat
                shared_data["currLong"] = lon
                pastGPSPositions.append((lat, lon))
                print("GPS RECEIVED!!")  
            else:
                print("FAIL")
        print("Light:::", shared_data["lightOn"])
        if(shared_data["manualOverride"]):
            if(lastLight != shared_data["lightOn"]):
                lastLight = shared_data["lightOn"]
                if(lastLight):
                    leftMotor.lightOn()
                    print("TURNING THE LIGHT ON!!")
                else:
                    leftMotor.lightOff()
                    
            if(shared_data["onLand"]):
                left = float(shared_data["speedLeft"])
                right = float(shared_data["speedRight"])
                if(last_ground_left != left):
                    last_ground__left = left
                    leftGroundMotor.sendCommand(left)
                if(last_ground__right != right):
                    rightGroundMotor.sendCommand(right)
                    last_ground__right = right
                if(last_left != 0.0):
                    leftMotor.sendCommand(0.0)
                    last_left = 0.0
                if(last_right != 0.0):
                    rightMotor.sendCommand(0.0)
                    last_right = 0.0
            else:
                left = float(shared_data["speedLeft"])
                right = float(shared_data["speedRight"])
                if(last_left != left):
                    last_left = left
                    leftMotor.sendCommand(left)
                if(last_right != right):
                    rightMotor.sendCommand(right)
                    last_right = right
                if(last_ground_left != 0.0):
                    leftGroundMotor.sendCommand(0.0)
                    last_ground_left = 0.0
                if(last_ground_right != 0.0):
                    rightGroundMotor.sendCommand(0.0)
                    last_ground_right = 0.0
                    
        else:
            distToTarget = calculate_distance(shared_data["target_lats"][0], shared_data["target_longs"][0], curr_lat, curr_long)
            if(distToTarget < 5):
                test1 = shared_data["target_lats"]
                test2 = shared_data["target_longs"]
                test1.pop()
                test2.pop()
                shared_data["target_lats"] = test1
                shared_data["target_longs"] = test2
                
            targetCoordinates = (0, 0)
            if(len(shared_data["target_lats"]) > 0):
                targetCoordinates = (shared_data["target_lats"][0], shared_data["target_longs"][0])
                
                thetaBias = 2.43
                print("GETTING THETA!!")
                #currentTheta = getSmoothAzimuth(sensor) - thetaBias
                print("CURRENT THETA: " + str(currentTheta))
                    
                #gains = [9.0, 2.0, 0.0, 0.0, math.radians(45.0), 0.0, 0.0, 0.5]  # [k1, k2, k3, k4, k5, k6, k7, k8]
                
                #pwmValues = thruster_control(targetCoordinates[0], targetCoordinates[1], gains, lat, lon, currentTheta)
                pwmValues = control_code(targetCoordinates[0], targetCoordinates[1], pastGPSPositions[0][0], pastGPSPositions[0][1], currentTheta)
                #rightMotor.sendCommand(pwmValues[0])
                #leftMotor.sendCommand(pwmValues[1])
            else:
                leftMotor.lightOn()
                # LIGHTS ON PACKAGE ARRIVED!!

import time
from multiprocessing import Manager, Process
from web_server import run_server

if __name__ == "__main__":
    try:
        # Initialize the Manager and shared dictionary
        manager = Manager()
        shared_data = manager.dict()
        shared_data["manualOverride"] = True
        shared_data["onLand"] = False
        shared_data["lightOn"] = False
        shared_data["command"] = "stop"
        shared_data["basicSpeed"] = 0
        shared_data["target_lats"] = collections.deque([0] * 5, maxlen=5)
        shared_data["target_longs"] = collections.deque([0] * 5, maxlen=5)
        shared_data["speedRight"] = 0
        shared_data["speedLeft"] = 0
        shared_data["currLat"] = 0
        shared_data["currLong"] = 0
        shared_data["targetLat"] = 0
        shared_data["targetLong"] = 0
        shared_data["batteryLevel"] = 100
        shared_data["heading_string"] = "NNN 0"
        
        print(list(shared_data["target_lats"]))
        print(list(shared_data["target_longs"]))
        
        
        # Start worker and web server processes
        worker_process = Process(target=main, args=(shared_data,))
        server_process = Process(target=run_server, args=(shared_data,))

        worker_process.start()
        server_process.start()

        # Wait for processes to finish
        worker_process.join()
        server_process.join()
        #main()
    except KeyboardInterrupt:
        print("Process interrupted.")
        cleanupThrusters()
        cleanupGPS()


# if __name__ == "__main__":
#     # Initialize the Manager and shared dictionary
#     manager = Manager()
#     shared_data = manager.dict()
#     shared_data["message"] = "Worker starting up..."
#     shared_data["target_coordinates"] = collections.deque(maxlen=5)
# 
#     # Start worker and web server processes
#     worker_process = Process(target=worker_task, args=(shared_data,))
#     server_process = Process(target=run_server, args=(shared_data,))
# 
#     worker_process.start()
#     server_process.start()
# 
#     # Wait for processes to finish
#     worker_process.join()
#     server_process.join()
