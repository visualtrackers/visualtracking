#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Usage:
1. ssh into the companion computer using the -Y argument:  ssh -Y companionComputer@ipAddress
2. Install the necessary dependencies (specified in the import list below)
3. Run the command: python3 visual_tracker.py --connect <*connection_string>
    example: "python3 visual_tracker.py -- connect :14550"


This script connects to the drone and waits until armed. When armed it will takeoff to the altitude specified by TARGET_ALTITUDE (set to 8 meters) and faces north.
Then when control is given it will start tracking and moving with the object selected.

Once the drone is in the air, a camera feed will start on the computer that is ssh-ed into the companion computer so you can see what the drone is seeing for control.

You need to line up the object you are trying to track in the center of the frame. Then, you need to press the 's' key. This stands for "start" and it will lock onto the object in the center of the frame and start tracking it around.
It will also move the drone to follow the object. The way it does this is by dividing the frame up into 9 boxes. If the object is in the center box, it sends a Go-To command to the current location.
If the object is in the top center box, it sends a Go-TO command a certain number of meters north (specified by the variable responseWeight). If the object is in the top right box of the frame, it will go responseWeight meters north and responseWeight meters east, etc.
To lose the bounding box, press the 'l' key, which means that you "lost" the object and no longer want it to track it anymore. This also stops the drone.
To stop the flight, press the 'q' button which "quits" the program, returning the drone to launch, and then ending.

Command Summary:
    s: get lock on object in center of frame and start moving around tracking that object
    l: stop tracking the object in the center of the frame and stop the drone moving around
    d: start moving the drone based on the object already aquired a lock on in 's' (this is depreciated and not necessary)
    x: stop moving the drone based on object being tracked (this is depreciated and not necssary)
    q: return to launch and stop the script



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
import os
import argparse # Necessary for parsing argument (connection string) passed into script


"""
Variables

# These variables are necessary for the drone's control
"""

# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 8 # Target altitude is now 8 meters up


iteration_time = 0.1 # This is how long the program stalls between iterations of the loop
current = [0, 0]    # This variable holds the coordinates of the center of the bounding box of the object being tracked. It is initialized to zero, zero because the object should be near the center at the start.
trackerWorking = True # This value is used in order to see whether the tracker is properly working. If the tracker ever has an issue (e.g. loses the bounding box) this value is set to False
droneStopped = False # This value is used in order to know if the drone is in the stopped position. If this is false and the execution needs to stop the drone, it will stop the drone and set this True
zone = 4        # This value is used to store which zone the object being tracked is currently in. The zones are numbered from 0-9 from top left to bottom right, so 4 is the center box
controlVehicle = False # This value is used to tell whether the code should be controlling the drone at any point. If this is set to True, then that means the code will issue Go-To commands

# Define an initial bounding box
bbox = (250, 175, 100, 100) # This is the size of the bounding box that is placed on the screen to track the object within it when the 'S' key is pressed. It is close to the center of the screen, but not perfect
 
target_yaw = 0 # This sets the angle you want the drone to face. For our code to work it needs to be 0 which equals North, but to change it to another use  target_yaw = math.degrees(90) # This sets yaw to 90 degrees, so East
responseWeight = 2 # This value is how many meters each of the Go-To commands sent while the code is in control are. It being 2 meters (so the drone would move in increments of 2 meters in each direction) worked best for keeping the object tracking by not moving too fast, but also keeping up with a brisk walking pace of a person being tracked






#############
# Functions and variables provided by Dr. Sichitiu in square_off.py:

# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False

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
    
    
# This function is used to the set the yaw any direction you want, but we only use it to set the yaw to north for our project
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

# End of functions given by Dr. Sichitiu in square_off.py
###########################


#########################
# Code for receiving input over terminal

# These imports are necessary for receiving keyboard input via the terminal
# Windows
if os.name == 'nt':
    import msvcrt

# Posix (Linux, OS X)
else:
    import termios
    import atexit
    from select import select


# This class is necessary for receiving keyboard input via the terminal
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
kb = KBHit() # This initialization is required to read in terminal input

# End of code required to read in input from the terminal
######################################






#############################################################
# My function definitions:

'''
This function is called when the code wants to track the next iteration of where the object is.
It takes in the frame of video that has been read in directly before, calls the tracker to get the new bounding box, then does calculations to see where the center of the bounding box is, and then returns the center coordinates and the bounding box so that the bounding box can be displayed on the livestream
'''
def trackFrame(frame):
      
    # Call the tracker object so it will return the new bounding box that surrounds where the object is in the frame at the moment this function is called
    ok, bbox = tracker.update(frame)

    # If the tracker call had no errors and returned a valid bounding box, then handle a tracking success:
    if ok:
        # Tracking success
        
        trackerWorking = True # This boolean indicates a tracking success to the code
        
        # Get the x and y locations of the object in the frame. This is the math to get the center coordinates of the bounding box
        # The bounding box object is a four number array that holds two coordinate positions: the top left of the box and the bottom right of the box.
        # Therefore, a bounding box looks like this bbox = [x_top-left, y_top-left, x_bottom-right, y_bottom_right]
        object_location_x = int((2*bbox[0] + bbox[2]) / 2)  # x value of center of bounding box: top left x plus bottom right x divided by two
        object_location_y = int((2*bbox[1] + bbox[3]) / 2)  # y value of center of bounding box: top left y plus bottom right y divided by two
        
    
    # If the tracker call had an error, then that means it lost the object, so we need to handle a tracker failure
    else :
        # Tracking failure
        print("Tracking ERROR: Boolean trackerWorking now set false.")
        trackerWorking = False


    return object_location_x, object_location_y, bbox # Return the coordinates of the center of the bounding box, and the bounding box itself so it can be displayed on the screen




###################################################################




###############
# This is connection code given in square_off.py in order to connect to the drone for control

# Set up option parsing to get connection string and mission plan file
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# Aquire connection_string. This is what is passed in with the parameter '--connect'
connection_string = args.connect

# Exit if no connection string specified. If we're not
if not connection_string:
    sys.exit('Please specify connection string. E.g. if your port number for drone control is 14550:   python3 visual_tracker.py --connect :14550')

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

# End of code that connects and waits for the drone to arm
######################




######################
# Now from here and below, the drone has connected, and we are ready to start our own code.


# The first step is to read in the video. This starts the feed earlier so it has time to adjust exposure automatically via the webcam
video = cv2.VideoCapture(0) # The number inside this can change depending on which camera feeds you have set up. Usually, external USB webcames are the lowest number 0, but if you get an error try 1, 2, etc.


# This section ensure that the joystick is set to the neutral position for at least two seconds and takes off before continuing on with our code.
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

    # Takeoff to short altitude set by TARGET_ALTITUDE at the top of the file
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            print("About to break out of takoff while loop")
            break
        time.sleep(0.5)
        
    # Set the yaw to north:
    condition_yaw(0)
    
    
    
    

##################
# Now at this point the drone is in the air and facing north!




 
 
# Initialize the tracker used for object tracking
# There are many more styles of trackers already imported if you would like to try another one
# Check out the "tracker_test" jupyter notebook for a list of the trackers and their names
tracker = cv2.TrackerCSRT_create()


# If the video had an error opening (e.g., no camera connected) then land the drone and exit the script!
if not video.isOpened():
    print("Could not open video")
    
    # Land the drone:
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
    
    # Exit the script, which stops all execution
    sys.exit()





# These three lines just issue a simple Go-To to the current location
# This ensures that the drone stays in place while it waits 1 second just to ensure that everything is working smoothly
currentLocation = vehicle.location.global_relative_frame
targetLocation = get_location_metres(currentLocation, 0, 0, TARGET_ALTITUDE)
vehicle.simple_goto(targetLocation)

# Print that there will be a delay, and then delay
print("Video feed will begin in one second...")
time.sleep(1)



# This is a try statement that will catch a keyboard interrupt (^c) being pressed on the terminal to stop execution of the program
try:

    # This is the main while loop that executes until the code is done
    while True:
    
        # The first stopo is to read in the video feed from the camera
        ok, frame = video.read()

        try:
            x_val, y_val, bbox = trackFrame(frame)  # Pass in that video frame just read in, and perform the tracking on it to see where the object is!
            current = [x_val, y_val] # Set the current position variable to these coordinates read in
            print(current) # Print the coordinates so the GCS can see where the object is in the frame
        
        except:
            # Handle a tracking error, stop the tracker, and also reset x_val and y_val
            print("Bounding Box not Found!")
            trackerWorking = False
            x_val = 0
            y_val = 0
            
        # Then show the bounding box and the frame on the companion computer:
        p1 = (int(bbox[0]), int(bbox[1])) # Get the top left corner of the bounding box
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])) # Get the bottom right corner of the bounding box
        cv2.rectangle(frame, p1, p2, (0,0,255), 2, 1) # Add a rectangle on the frame on the companion computer ssh-ed in to show where the bounding box is
        cv2.imshow("Tracking", frame) # Then show the actual frame read in from the video feed

        
        # This next portion gets the divisions for dividing the frame up into the nine boxes.
        # We made them all be equidistant, as making them not at 1/3 and 2/3 but at 2/5 and 3/5 did not work, but you could experiment with other vertical and horizontal divisions
        x_divs = [frame.shape[0] / 3, (2 * frame.shape[0] / 3)] # Set the two x (vertical) divisions to be 1/3 and 2/3 of the way through the frame
        y_divs = [frame.shape[1] / 3, (2 * frame.shape[1] / 3)] # Set the two y (horizontal) divisions to be 1/3 and 2/3 of the way through the frame


        # Now this next big block of nested if statments is used to figure out which zone the object is in
        # It is long and tedious, but we wanted it to be this way so we could change the divisions above and still accurately calculate zones.

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
                print("Now in zone 4 - The object is center frame!")
            else:
                # bottom middle
                zone = 7
                print("Now in zone 7")




        # Now this next block takes in the zone calculated above and issues the appropriate go-to command of responseWeight in the correct directions
        # It only does this if the tracker is working though
        if (trackerWorking == True):
        
            print("Issueing go-to command in specific direction....")
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
            
            
        # If the tracker is not working, set that I should stop controlling the vehicle and issue a go-to command to the current location
        else:
            controlVehicle = False
            print("controlVehicle set false!")
            if (droneStopped == False):
                currentLocation=vehicle.location.global_relative_frame
                targetLocation=get_location_metres(currentLocation, 0, 0, TARGET_ALTITUDE)
                vehicle.simple_goto(targetLocation)
                droneStopped = True



        

        # This function looks redundant to below, and it kind of is
        # But IT IS VERY NECESSARY because it keeps the openCV window open on the companion computer so you can see the video!
        if cv2.waitKey(1) & 0xFF == ord('q'): # press 'q' to return to lauch!
            vehicle.mode = VehicleMode("RTL")
            print("RETURN TO LAUNCH KEY PRESSED!!")




        
        # Below is where the script checks for input from the terminal.
        # These are the actions of the keypresses:
        '''
        Command Summary:
            s: get lock on object in center of frame and start moving around tracking that object
            l: stop tracking the object in the center of the frame and stop the drone moving around
            d: start moving the drone based on the object already aquired a lock on in 's' (this is depreciated and not necessary)
            x: stop moving the drone based on object being tracked (this is depreciated and not necssary)
            q: return to launch and stop the script
        '''
        
        if (kb.kbhit()):
                c = kb.getch()
                
                # c has the character read in
                
                if (c == 'd'):
                    controlVehicle = True
                    print("controlVehicle has been set True")
                    
                if (c == 'x'):
                    controlVehicle = False
                    print("controlVehicle has been set False")
                    
                if (c == 's'):
                    print("Setting a new bounding box!")
                    # Define a bounding box
                    bbox = (250, 175, 100, 100) #changed box size
                    
                    # Initialize tracker with bounding box
                    tracker = cv2.TrackerCSRT_create()
                    ok = tracker.init(frame, bbox)
                    trackerWorking = True
                    droneStopped = False
                    controlVehicle = True
                    print("controlVehicle has been set True")
                    
                if (c == 'q'):
                    vehicle.mode = VehicleMode("RTL")
                    print("RETURN TO LAUNCH KEY PRESSED!!")
                    
                if (c == 'l'):
                    trackerWorking = False
                    droneStopped = False
                    print("Manually killed the bounding box!!")

 

            
        # Now I want to sleep the loop for the iteration time before starting the next loop iteration
        # This ensures that the loop doesn't execute too fast to keep up with the object
        time.sleep(iteration_time)
        
        
        # End of the while loop
        ################


# This is the
except KeyboardInterrupt:
    print('exiting')
    video.release()
    pass




# Then this is Sichitu's code for landing, which we want to do when our code is done running:

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
