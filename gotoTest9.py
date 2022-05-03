#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
usage: python square_off.py --connect <*connection_string>
This script connects to the drone and waits until armed. When armed it will takeoff
to 3m altitude, then navigate a 10x10 meter square. At each corner of the square the drone
will wait for 5 seconds.
"""

from __future__ import print_function

import math
import time
import sys
from unittest import case
from dronekit import connect, VehicleMode, LocationGlobalRelative
import dronekit
from pymavlink import mavutil
import cv2
import sys
import time

# Code for receiving input over terminal
import os
import time

# Windows
if os.name == 'nt':
    import msvcrt

# Posix (Linux, OS X)
else:
    import sys
    import termios
    import atexit
    from select import select

class KBHit:

    def __init__(self):
        '''Creates a KBHit object that you can call to do various keyboard things.
        '''

        if os.name == 'nt':
            pass

        else:

            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)

            # New terminal setting unbuffered
            self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)


    def set_normal_term(self):
        ''' Resets to normal terminal.  On Windows this is a no-op.
        '''

        if os.name == 'nt':
            pass

        else:
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)


    def getch(self):
        ''' Returns a keyboard character after kbhit() has been called.
            Should not be called in the same program as getarrow().
        '''

        s = ''

        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')

        else:
            return sys.stdin.read(1)


    def getarrow(self):
        ''' Returns an arrow-key code after kbhit() has been called. Codes are
        0 : up
        1 : right
        2 : down
        3 : left
        Should not be called in the same program as getch().
        '''

        if os.name == 'nt':
            msvcrt.getch() # skip 0xE0
            c = msvcrt.getch()
            vals = [72, 77, 80, 75]

        else:
            c = sys.stdin.read(3)[2]
            vals = [65, 67, 66, 68]

        return vals.index(ord(c.decode('utf-8')))


    def kbhit(self):
        ''' Returns True if keyboard character was hit, False otherwise.
        '''
        if os.name == 'nt':
            return msvcrt.kbhit()

        else:
            dr,dw,de = select([sys.stdin], [], [], 0)
            return dr != []


# Size of square in meters
SQUARE_SIZE = 10
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 8 # Target altitude is now 15 feet up
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    inityawmsg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(inityawmsg)
    
    time.sleep(1)
    print("Getting the gimbal in place!")


    # delay to wait until yaw of copter is at desired yaw angle and gimbal is set to appropriate bearing
    time.sleep(3)

# Set up option parsing to get connection string and mission plan file
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

print('Succesfully connected to vehicle')

"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

# Wait for pilot before proceeding
print('Waiting for safety pilot to arm...')


# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('Armed...')
vehicle.mode = VehicleMode("GUIDED")

######################
# Move camera initialization here
# Read video
video = cv2.VideoCapture(0) # for using CAM


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:

    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)
    
    # Takeoff to short altitude
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            print("About to break out of takoff while loop")
            break
        time.sleep(0.5)
    # yaw north
    condition_yaw(0)
    
    


def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]


# Put code for what to do once in the air:

"""
Variables

##Set up these values before flying!!!!
"""
run = 1
setpoint_x = 0
setpoint_y = 0
setpoint = [setpoint_x, setpoint_y]
Kp = 0.2
Ki = 0               
Kd = 0
integral = [0.0, 0.0]
error_prior = [0.0, 0.0]
iteration_time = .1
current = [0, 0]
bias = 0
hover_thrust = 0.5
trackerWorking = True
selectBB = False
droneStopped = False
secondCounter = 0
loopCounter = 0
zone = 4




controlVehicle = False 

 
 
#     # Set up tracker.
tracker = cv2.TrackerCSRT_create()


# Exit if video not opened.
if not video.isOpened():
    print("Could not open video")
    
    print('Landing')
    if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
        # Land Copter
        vehicle.mode = VehicleMode("LAND")

    # Stay connected to vehicle until landed and disarmed
    while vehicle.armed:
        time.sleep(1)

    print("Done!")

    # Close vehicle object before exiting script
    vehicle.close()
    
    sys.exit()


# Read third frame.
# ok, frame = video.read() # Commented out because redundant


# if not ok:
#     print ('Cannot read video file')
#     sys.exit()                                  # Does script crash during this command??



# Define an initial bounding box
bbox = (250, 175, 100, 100) #changed box size


#############################################################

def trackFrame(frame, bbox):
  
      
    # Update tracker
    ok, bbox = tracker.update(frame)

    # Draw bounding box
    if ok:
        # Tracking success
        trackerWorking = True
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        object_location_x = int((2*bbox[0] + bbox[2]) / 2)
        object_location_y = int((2*bbox[1] + bbox[3]) / 2)
        # cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        dimensions = frame.shape
        x_image_width = dimensions[1]
        y_image_width = dimensions[0]
        x_error = x_image_width / 2 - object_location_x
        y_error = y_image_width / 2 - object_location_y
        
        cv2.imshow("Tracking", frame)


    else :
        # Tracking failure
        print("Tracking error! trackerWorking set false!")
        trackerWorking = False


    # return x_error, y_error, bbox
    return object_location_x, object_location_y, bbox

def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, altitude)
###################################################################

target_yaw = 0 #math.degrees(90)

currentLocation=vehicle.location.global_relative_frame
targetLocation=get_location_metres(currentLocation, 0, 0, TARGET_ALTITUDE)
vehicle.simple_goto(targetLocation)

print("Control begin in three seconds...")
time.sleep(3)

kb = KBHit()


responseWeight = 2

 '''main tracking algorithm''' 
try:       
    while run:       '''disply video feed on the GCS + the bouding box''' 
        ok, frame = video.read()
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (0,0,255), 2, 1)
        cv2.imshow("Tracking", frame)

        try:
            x_val, y_val, bbox = trackFrame(frame, bbox)
            current = [x_val, y_val]
            print(current)
        
        except:
            print("Bounding Box not Found!")
            trackerWorking = False
            x_val = 0
            y_val = 0

        
        x_divs = [frame.shape[0] / 3, (2 * frame.shape[0] / 3)]
        y_divs = [frame.shape[1] / 3, (2 * frame.shape[1] / 3)]


        #Defines the video zones so we know where the bounding box is on the video feed
        if(x_val > x_divs[1]): # right in x axis
            if(y_val < y_divs[0]):
                # top right
                print("Now in zone 2")
                zone = 2
            elif(y_val < y_divs[1]):
                # middle right
                zone = 5
                print("Now in zone 5")
            else:
                # bottom right
                zone = 8
                print("Now in zone 8")
        elif(x_val < x_divs[0]): #left in x axis
            if(y_val < y_divs[0]):
                # top left
                zone = 0
                print("Now in zone 0 ")
            elif(y_val < y_divs[1]):
                # middle left
                zone = 3
                print("Now in zone 3")
            else:
                # bottom left
                zone = 6
                print("Now in zone 6")
        else: # middle of x
            if(y_val < y_divs[0]):
                # top middle
                zone = 1
                print("Now in zone 1")
            elif(y_val < y_divs[1]):
                # middle middle (SAFE ZONE)
                zone = 4
                print("Now in zone 4 = no action taken because center frame!!")
            else:
                # bottom middle
                zone = 7
                print("Now in zone 7")

        #Goto functions that moves the UAV accordingly to maintain the bounding box in the center of the video feed
        if (trackerWorking == True):
            print("One second elapsed... issuing a goto command!")
            if(zone == 0):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, responseWeight, -responseWeight, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 1):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, responseWeight, 0, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 2):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, responseWeight, responseWeight, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 3):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, 0, -responseWeight, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 4):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, 0, 0, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 5):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, 0, responseWeight, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 6):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, -responseWeight, -responseWeight, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 7):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, -responseWeight, 0, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            elif(zone == 8):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, -responseWeight, responseWeight, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
            
                
        else:
            controlVehicle = False
            print("controlVehicle set false!")
            if (droneStopped == False):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, 0, 0, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
                droneStopped = True


        if cv2.waitKey(1) & 0xFF == ord('d'): # press 'd' to allow self control, UAV will maintain bouding box in the center of video
            controlVehicle = True
            print("controlVehicle has been set True")

        if cv2.waitKey(1) & 0xFF == ord('x'): # press 'x' to stop self control
            controlVehicle = False
            print("controlVehicle has been set False")
        

        if cv2.waitKey(1) & 0xFF == ord('s'): # press 's' to inititalize tracker
            print("Setting a new bounding box!")
            # Define a bounding box
            bbox = (250, 175, 100, 100) #changed box size
            #bbox = (int(x_divs[0]), int(y_divs[0]), int(y_divs[1] - y_divs[0]), int(x_divs[1] - x_divs[0]))
            # Initialize tracker with bounding box
            tracker = cv2.TrackerCSRT_create()
            ok = tracker.init(frame, bbox)
            trackerWorking = True
            droneStopped = False

        
        if cv2.waitKey(1) & 0xFF == ord('q'): # press 'q' to return to lauch!
            vehicle.mode = VehicleMode("RTL")
            print("RETURN TO LAUNCH KEY PRESSED!!")  

        if cv2.waitKey(1) & 0xFF == ord('l'): # press 'l' to lose bounding box manually!
            trackerWorking = False
            droneStopped = False
            print("Manually killed the bounding box!!")     

        loopCounter = loopCounter + 1

        if (kb.kbhit()):        #Use input from terminal which is more accurate than cv2.waitKey
                c = kb.getch()
                
                # c has the character read in
                
                if (c == 'd'):      # press 'd' to allow self control, UAV will maintain bouding box in the center of video
                    controlVehicle = True
                    print("controlVehicle has been set True")
                    
                if (c == 'x'):     # press 'x' to stop self control
                    controlVehicle = False
                    print("controlVehicle has been set False")
                    
                if (c == 's'):      # press 's' to inititalize tracker
                    print("Setting a new bounding box!")
                    # Define a bounding box
                    bbox = (250, 175, 100, 100) #changed box size
                    #bbox = (int(x_divs[0]), int(y_divs[0]), int(y_divs[1] - y_divs[0]), int(x_divs[1] - x_divs[0]))
                    # Initialize tracker with bounding box
                    tracker = cv2.TrackerCSRT_create()
                    ok = tracker.init(frame, bbox)
                    trackerWorking = True
                    droneStopped = False
                    controlVehicle = True
                    print("controlVehicle has been set True")
                    
                if (c == 'q'):      # press 'q' to return to lauch!
                    vehicle.mode = VehicleMode("RTL")
                    print("RETURN TO LAUNCH KEY PRESSED!!")  
                    
                if (c == 'l'):      # press 'l' to lose bounding box manually!
                    trackerWorking = False
                    droneStopped = False
                    print("Manually killed the bounding box!!") 

        # if (loopCounter == 10) :
        #     secondCounter  = secondCounter + 1
        #     loopCounter = 0
        #     print("One second has elapsed!")

            
        time.sleep(iteration_time)

except KeyboardInterrupt:
    print('exiting')
    video.release()
    pass



# Then this is Sichitu's code for landing

print('Landing')
if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
    # Land Copter
    vehicle.mode = VehicleMode("LAND")



# Stay connected to vehicle until landed and disarmed
while vehicle.armed:
    time.sleep(1)

print("Done!")

# Close vehicle object before exiting script
vehicle.close()
