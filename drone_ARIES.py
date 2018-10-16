import time
import cv2
import math
from dronekit import *
from pymavlink import mavutil
import tkinter as tk
from people_detect_haar import detect_return

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
	  print("waiting to be armable")
	  time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
	  v_alt = vehicle.location.global_relative_frame.alt
	  print(">> Altitude = %.1f m"%v_alt)
	  if v_alt >= altitude - 1.0:
		  print("Target altitude reached")
		  break
	  time.sleep(1)

def clear_mission(vehicle):
	"""
	Clear the current mission.
	"""
	cmds = vehicle.commands
	vehicle.commands.clear()
	vehicle.flush()

	# After clearing the mission you MUST re-download the mission from the vehicle
	# before vehicle.commands can be used again
	# (see https://github.com/dronekit/dronekit-python/issues/230)
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready()

def download_mission(vehicle):
	"""
	Download the current mission from the vehicle.
	"""
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready() # wait until download is complete.
	

def get_current_mission(vehicle):
	"""
	Downloads the mission and returns the wp list and number of WP 
	
	Input: 
		vehicle
		
	Return:
		n_wp, wpList
	"""

	print ("Downloading mission")
	download_mission(vehicle)
	missionList = []
	n_WP        = 0
	for wp in vehicle.commands:
		missionList.append(wp)
		n_WP += 1 
		
	return n_WP, missionList
	

def add_last_waypoint_to_mission(                                       #--- Adds a last waypoint on the current mission file
		vehicle,            #--- vehicle object
		wp_Last_Latitude,   #--- [deg]  Target Latitude
		wp_Last_Longitude,  #--- [deg]  Target Longitude
		wp_Last_Altitude):  #--- [m]    Target Altitude
	"""
	Upload the mission with the last WP as given and outputs the ID to be set
	"""
	# Get the set of commands from the vehicle
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready()

	# Save the vehicle commands to a list
	missionlist=[]
	for cmd in cmds:
		missionlist.append(cmd)

	# Modify the mission as needed. For example, here we change the
	wpLastObject = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
						   wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
	missionlist.append(wpLastObject)

	# Clear the current mission (command is sent when we call upload())
	cmds.clear()

	#Write the modified mission and flush to the vehicle
	for cmd in missionlist:
		cmds.add(cmd)
	cmds.upload()
	
	return (cmds.count)    

def ChangeMode(vehicle, mode):
	while vehicle.mode != VehicleMode(mode):
			vehicle.mode = VehicleMode(mode)
			time.sleep(0.5)
	return True

#-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
	""" Remember: vz is positive downward!!!
	http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
	
	Bitmask to indicate which dimensions should be ignored by the vehicle 
	(a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
	none of the setpoint dimensions should be ignored). Mapping: 
	bit 1: x,  bit 2: y,  bit 3: z, 
	bit 4: vx, bit 5: vy, bit 6: vz, 
	bit 7: ax, bit 8: ay, bit 9:	
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
			0,
			0, 0,
			mavutil.mavlink.MAV_FRAME_BODY_NED,
			0b0000111111000111, #-- BITMASK -> Consider only the velocities
			0, 0, 0,        #-- POSITION
			vx, vy, vz,     #-- VELOCITY
			0, 0, 0,        #-- ACCELERATIONS
			0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
	
	
#-- Key event function for manual control via keyboard keys
def key(event):
	if event.char == event.keysym: #-- standard keys
		if event.keysym == 'r':
			print("r pressed >> Set the vehicle to RTL")
			vehicle.mode = VehicleMode("RTL")
			
	else: #-- non standard keys
		if event.keysym == 'Up':
			set_velocity_body(vehicle, gnd_speed, 0, 0)
		elif event.keysym == 'Down':
			set_velocity_body(vehicle,-gnd_speed, 0, 0)
		elif event.keysym == 'Left':
			set_velocity_body(vehicle, 0, -gnd_speed, 0)
		elif event.keysym == 'Right':
			set_velocity_body(vehicle, 0, gnd_speed, 0)

#-- Setting ROI loiter around a location
def set_roi(location):
	"""
	Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
	specified region of interest (LocationGlobal).
	The vehicle may also turn to face the ROI.

	For more information see: 
	http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
	"""
	# create the MAV_CMD_DO_SET_ROI command
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
		0, #confirmation
		0, 0, 0, 0, #params 1-4
		location.lat,
		location.lon,
		location.alt
		)
	# send command to vehicle
	vehicle.send_mavlink(msg)

#-- Returns the ground distance in metres between two LocationGlobal objects.
def get_distance_metres(aLocation1, aLocation2):
	"""
	This method is an approximation, and will not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlong = aLocation2.lon - aLocation1.lon
	return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

#-- Returns the bearing between the two LocationGlobal objects passed as parameters.
def get_bearing(aLocation1, aLocation2):
	"""
	This method is an approximation, and may not be accurate over large distances and close to the 
	earth's poles. It comes from the ArduPilot test code: 
	https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
	"""	
	off_x = aLocation2.lon - aLocation1.lon
	off_y = aLocation2.lat - aLocation1.lat
	bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
	if bearing < 0:
		bearing += 360.00
	return bearing;


#-- Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
def goto(dNorth, dEast):
	"""
	The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
	the target position. This allows it to be called with different position-setting commands. 
	By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

	The method reports the distance to target every two seconds.
	"""
	
	currentLocation = vehicle.location.global_relative_frame
	targetLocation = get_location_metres(currentLocation, dNorth, dEast)
	targetDistance = get_distance_metres(currentLocation, targetLocation)
	gotoFunction(targetLocation)

	while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
		#print "DEBUG: mode: %s" % vehicle.mode.name
		remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
		print "Distance to target: ", remainingDistance
		if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
			print "Reached target"
			break;
		time.sleep(2)

#-- Removes the WPs from mission list that have been covered
def restart_mission():
	# # Get the set of commands from the vehicle
	# cmds = vehicle.commands
	# cmds.download()
	# cmds.wait_ready()

	# # Save the vehicle commands to a list
	# curr_wp = cmds.next - 1
	# missionlist=[]
	# i = 0
	# for cmd in cmds:
	# 	i = i+1
	# 	if i <= curr_wp:
	# 		pass
	# 	else:
	# 		missionlist.append(cmd)

	# # Clear the current mission (command is sent when we call upload())
	# cmds.clear()

	# #Write the modified mission and flush to the vehicle
	# for cmd in missionlist:
	# 	cmds.add(cmd)
	# cmds.upload()
	ChangeMode(vehicle,"AUTO")


#-- Pause the current mission
def mission_pause():
	# Change the mode to GUIDED
	ChangeMode(vehicle,"GUIDED")
	# Stop the vehicle by giving 0 velocity
	set_velocity_body(vehicle, 0, 0, 0)

#-- People detection using HAAR
def do_people_detect():
	detected = False
	r, frame = cap.read()
	if r:
		frame = cv2.resize(frame,(640,360)) # Downscale to improve frame rate
		gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) # Haar-cascade classifier needs a grayscale image
		rects = person_cascade.detectMultiScale(gray_frame)       
		for (x, y, w, h) in rects:
			if x:
				detected = True			
			cv2.rectangle(frame, (x,y), (x+w,y+h),(0,255,0),2)
		cv2.imshow("preview", frame)
		return detected


#--------------------------------------------------
#-------------- INITIALIZE  
#--------------------------------------------------      
#-- Setup the commanded flying speed
gnd_speed = 5 # [m/s]
mode      = 'GROUND'
#-- Setup people detection
person_cascade = cv2.CascadeClassifier('haarcascade_fullbody.xml')
cap = cv2.VideoCapture(0)

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')
#vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

#--------------------------------------------------
#-------------- MAIN FUNCTION  
#--------------------------------------------------    
while True:
	
	if mode == 'GROUND':
		#--- Wait until a valid mission has been uploaded
		n_WP, missionList = get_current_mission(vehicle)
		time.sleep(2)
		if n_WP > 0:
			print ("A valid mission has been uploaded: takeoff!")
			mode = 'TAKEOFF'
			
	elif mode == 'TAKEOFF':
	   
		#-- Add a fake waypoint at the end of the mission
		add_last_waypoint_to_mission(vehicle, vehicle.location.global_relative_frame.lat, 
									   vehicle.location.global_relative_frame.lon, 
									   vehicle.location.global_relative_frame.alt)
		print("Home waypoint added to the mission")
		time.sleep(1)
		#-- Takeoff
		arm_and_takeoff(10)
		
		#-- Change the UAV mode to AUTO
		print("Changing to AUTO")
		ChangeMode(vehicle,"AUTO")
		
		#-- Change mode, set the ground speed
		vehicle.groundspeed = gnd_speed
		mode = 'MISSION'
		print ("Switch mode to MISSION")
		
	elif mode == 'MISSION':
		#-- Here we just monitor the mission status. Once the mission is completed we go back
		#-- vehicle.commands.cout is the total number of waypoints
		#-- vehicle.commands.next is the waypoint the vehicle is going to
		#-- once next == cout, we just go home
		
		print ("Current WP: %d of %d "%(vehicle.commands.next, vehicle.commands.count))
		if vehicle.commands.next == vehicle.commands.count:
			print ("Final waypoint reached: go back home")
			#-- First we clear the flight mission
			clear_mission(vehicle)
			print ("Mission deleted")
			
			#-- We go back home
			ChangeMode(vehicle,"RTL")
			mode = "BACK"

		# detected = do_people_detect()
		detected = detect_return()
		if detected:
			mission_pause()
			# # set_roi(vehicle.location.global_relative_frame)
			# time.sleep(10)
			print "Detected"
			# #---------------------------------------------------------------------TODO Send GPS cordinates here
			restart_mission()
			
			
	elif mode == "BACK":
		if vehicle.location.global_relative_frame.alt < 1:
			print ("Switch to GROUND mode, waiting for new missions")
			mode = 'GROUND'

	k = cv2.waitKey(1)
	if k & 0xFF == ord("q"): # Exit condition
		break
	
	# time.sleep(0.5)

	# #- Read the keyboard with tkinter
	# root = tk.Tk()
	# print(">> Control the drone with the arrow keys. Press r for RTL mode")
	# root.bind_all('<Key>', key)
	# root.mainloop()

