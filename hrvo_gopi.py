import math
from easygopigo3 import EasyGoPiGo3
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys
import re

#Broadcast bot information
def send_data(conn):
	SERVER_IP   = '192.168.43.12'
	PORT_NUMBER = 5000
	SIZE = 1024

	mySocket = socket( AF_INET, SOCK_DGRAM )
	a=[60.0,0.0,0.0,0.0]
	while True:
		data = conn.recv()
	        mySocket.sendto(str(data),(SERVER_IP,PORT_NUMBER))
		time.sleep(0.5)	

		
#Read Neighboring bot information
def get_readings(my_distance_sensor):
	PORT_NUMBER = 5000
	SIZE = 1024

	hostName = gethostbyname( '0.0.0.0' )

	mySocket = socket( AF_INET, SOCK_DGRAM )
	mySocket.bind( (hostName, PORT_NUMBER) )

	print ("Test server listening on port {0}\n".format(PORT_NUMBER))

	while True:
		(data,addr) = mySocket.recvfrom(SIZE)
		print data
		result = [float(x) for x in re.findall('\d+[.]?\d*', data )]
		print result[0], result[1], result[2]
		break
	mySocket.close()
	return result
	#return my_distance_sensor.read()


def cmps_to_dps(speed):
	return speed * 360.0/21.0


def dps_to_cmps(speed):
	return speed * 21.0/360.0


#Read odometric values in cmps
def read_odom(gpg, ori):
	dps_speed = gpg.get_speed()
	cmps_speed = dps_to_cmps(dps_speed)
	vel_x = cmps_speed * math.cos(ori * PI/180.0)
	vel_y = cmps_speed * math.sin(ori * PI/180.0)
	return [vel_x, vel_y]


def get_dist(source_x, source_y, neighbor_x, neighbor_y):
	return math.sqrt(pow(source_x - neighbor_x, 2) + pow(source_y - neighbor_y, 2))

#Find angle between 2 points in 0:360 range
def get_rel_ang(source_x, source_y, neighbor_x, neighbor_y):
	PI = 3.14
	if((source_x - neighbor_x) == 0.0 and (source_y - neighbor_y) != 0.0):
		if((neighbor_y - source_y) > 0.0):
			return 90.0
		else:
			return 270.0
	angle = math.atan((neighbor_y - source_y)/(neighbor_x - source_x)) * 180.0/PI
	if(neighbor_x < source_x):
		angle = 180.0 + angle
	elif((neighbor_y < source_y) and (neighbor_x > source_x)):
		angle = 360.0 + angle
	return angle


#Update neighboring robot information- (distance, angle)
def update_bot_info(readings, ori, bot_pos_x, bot_pos_y):
	dist_vec = []
	angle_vec = []
	radii = 30
	dist_vec.append(get_dist(bot_pos_x, bot_pos_y, readings[0], readings[1]))
	angle_vec.append(get_rel_ang(bot_pos_x, bot_pos_y, readings[0], readings[1]))		
	return [dist_vec, angle_vec]


#Find the target velocity using the clear path algorithm, given the HRVO collision cones
def clear_path(bot_pos_x, bot_pos_y, pref_vel_x, pref_vel_y, lines_x, lines_y, lines_angles):
	intersections_x = []
	intersections_y = []
	vel_a_x = bot_pos_x + pref_vel_x
	vel_a_y = bot_pos_y + pref_vel_y
	flag = 1
	PI = 3.14
	#Find projections of current velocity vector on collision cones
	for i in range(len(lines_x)):
		m1 = math.tan(lines_angles[i] * PI/180.0)
		m = -1.0 / m1
		x = ((m * vel_a_x) - (m1 * lines_x[i]) - vel_a_y + lines_y[i]) / (m - m1)
		y = (m * (x - vel_a_x)) + vel_a_y
		if(i % 2 == 0):
			ang = get_rel_ang(lines_x[i], lines_y[i], vel_a_x, vel_a_y)
			print("Ang= "+str(ang))
			if((ang < lines_angles[i] and ang > lines_angles[i+1]) or (ang < lines_angles[i]+360.0 and ang > lines_angles[i+1]+360.0) or (ang+360.0 < lines_angles[i] and ang+360.0 > lines_angles[i+1])):
				flag = 0
			
		intersections_x.append(x) 
		intersections_y.append(y)

	#Check if Preferred velocity vector is inside a collision cone
	if(flag == 1):
		print("PROCEED AT PREFERRED VELOCITY")
		target_vel_x = vel_a_x
		target_vel_y = vel_a_y
		return [math.sqrt(pow(target_vel_x - bot_pos_x, 2) + pow(target_vel_y - bot_pos_y, 2)), target_vel_x, target_vel_y]

	#Find intersection points of collision cones
	for i in range(len(lines_x)-1):
		m = math.tan(lines_angles[i] * PI/180.0)
		for j in range(i+1, len(lines_x)):
			m1 = math.tan(lines_angles[j] * PI/180.0)
			x = ((m * lines_x[i]) - (m1 * lines_x[j]) - lines_y[i] + lines_y[j]) / (m - m1)
			y = (m * (x - lines_x[i])) + lines_y[i]
			
			intersections_x.append(x)
			intersections_y.append(y)

	new_vel_x = 100000.0
	new_vel_y = 100000.0
	min_dist = get_dist(vel_a_x, vel_a_y, new_vel_x, new_vel_y)
	#Find new velocity outside all collision cones
	for i in range(len(intersections_x)):
		if( min_dist < get_dist(vel_a_x, vel_a_y, intersections_x[i], intersections_y[i])):
			continue
		else:
			outside = 1
			for j in range(0,len(lines_x)-1,2):
				if(intersections_x[i] == lines_x[j] and intersections_y[i] == lines_y[j]):
					continue
				angle_wrt_apex = get_rel_ang(lines_x[j], lines_y[j], intersections_x[i], intersections_y[i])
				limit_max = lines_angles[j]
				limit_min = lines_angles[j+1]
				if(limit_min < 0.0):
					limit_min += 360.0
					limit_max += 360.0
				if(limit_max < limit_min):
					limit_max += 360.0
			
				if((((limit_max - angle_wrt_apex) > 0.01) and ((angle_wrt_apex - limit_min) >  0.01)) or (((limit_max - angle_wrt_apex - 360.0) > 0.01) and ((angle_wrt_apex + 360.0 - limit_min) >  0.01))):
					outside = 0
					break
			if(outside == 1):
				new_vel_x = intersections_x[i]
				new_vel_y = intersections_y[i]
				min_dist = get_dist(vel_a_x, vel_a_y, intersections_x[i], intersections_y[i])
	
	if(new_vel_x == 100000.0):
		return [0.0, bot_pos_x, bot_pos_y]
	else:
		print("*****************CHANGE ORIENTATION***************")
		target_vel_x = new_vel_x
		target_vel_y = new_vel_y
		return [math.sqrt(pow(new_vel_x - bot_pos_x, 2) + pow(new_vel_y - bot_pos_y, 2)), target_vel_x, target_vel_y]


#Find HRVO collision cone induced between two robots
def HRVO(bot_pos_x, bot_pos_y, vel_x, vel_y, dist, angle, lines_x, lines_y, lines_angles, readings):
	radii = 30
	PI = 3.14
	#obs_x = bot_pos_x + dist * math.cos(angle)
	#obs_y = bot_pos_y + dist * math.sin(angle)
	if(dist < radii):
		dist = radii
	ang_diff = math.asin(radii/dist) * 180/PI
	vo_max = ang_diff + angle
	vo_min = angle - ang_diff
	margin = 3
	vo_radii = radii + radii + margin
	vel_a_x = bot_pos_x + vel_x
	vel_a_y = bot_pos_y + vel_y
	obs_vel_x, obs_vel_y = [readings[2], readings[3]] 
	#Apex of RVO cone
	apex_rvo_x = bot_pos_x + (obs_vel_x + vel_x) / 2.0
	apex_rvo_y = bot_pos_y + (obs_vel_y + vel_y) / 2.0
	#Apex of VO cone	
	apex_vo_x = bot_pos_x + obs_vel_x
	apex_vo_y = bot_pos_y + obs_vel_y
	vel_ang_wrt_b = get_rel_ang(apex_rvo_x, apex_rvo_y, vel_a_x, vel_a_y)
	vo_cent_ang = angle
	vo_cent_dist = dist
	#Collision cone angles
	vo_max_ang = (math.asin(vo_radii/max(vo_cent_dist, vo_radii)) * 180/PI) + vo_cent_ang + 10.0
	vo_min_ang = vo_cent_ang - (math.asin(vo_radii/max(vo_cent_dist, vo_radii)) * 180/PI) - 10.0
	if(((vel_ang_wrt_b > vo_max_ang) or (vel_ang_wrt_b < vo_min_ang))):
		return [lines_x, lines_y, lines_angles]
	else:
		#Apex of HRVO cone
		if( vel_ang_wrt_b < vo_cent_ang):
			print("RIGHT SIDE")
			m = math.tan((vo_min_ang) * PI/180.0)
			m1 = math.tan((vo_max_ang) * PI/180.0)
			apex_x = ((m * apex_rvo_x) - (m1 * apex_vo_x) - apex_rvo_y + apex_vo_y) / (m - m1)
			apex_y = (m * (apex_x - apex_rvo_x)) + apex_rvo_y
		else:
			print("LEFT SIDE")
			m = math.tan((vo_max_ang) * PI/180.0)
			m1 = math.tan((vo_min_ang) * PI/180.0)
			apex_x = ((m * apex_rvo_x) - (m1 * apex_vo_x) - apex_rvo_y + apex_vo_y) / (m - m1)
			apex_y = (m * (apex_x - apex_rvo_x)) + apex_rvo_y

		lines_x.append(apex_x)
		lines_y.append(apex_y)
		lines_angles.append(vo_max_ang)

		lines_x.append(apex_x)
		lines_y.append(apex_y)
		lines_angles.append(vo_min_ang)
		print(apex_x, apex_y, vo_min_ang, vo_max_ang)
		return [lines_x, lines_y, lines_angles]


#Check for obstacles using the HRVO algorithm and find new velocity using Clear Path algorithm 
def obstacle(bot_pos_x, bot_pos_y, vel_x, vel_y, target_pose_x, target_pose_y, dist_vec, angle_vec, pref_vel, readings):
	new_vel = 100.0; 
	lines_x = []
	lines_y = []
	lines_angles = []
	for i in range(len(dist_vec)):
		if(dist_vec[i] <= 50):
			lines_x, lines_y, lines_angles = HRVO(bot_pos_x, bot_pos_y, vel_x, vel_y, dist_vec[i], angle_vec[i], lines_x, lines_y, lines_angles, readings)
	print("HRVO Done")
	dist_2_dest = get_dist(bot_pos_x,  bot_pos_y, target_pose_x, target_pose_y)
	if(dist_2_dest == 0.0):
		pref_vel_x = 0.0
		pref_vel_y = 0.0
	else:
		pref_vel_x = -pref_vel * (bot_pos_x - target_pose_x) / dist_2_dest
		pref_vel_y = -pref_vel * (bot_pos_y - target_pose_y) / dist_2_dest
	if(len(lines_x) == 0):
		result=[pref_vel, bot_pos_x+pref_vel_x, bot_pos_y+pref_vel_y]
		return result
	print("Preferred Velocity X= "+str(pref_vel_x)+" Y= "+str(pref_vel_y))
	print("Preferred Angle= "+str(get_rel_ang(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y)))      
	result = clear_path(bot_pos_x, bot_pos_y, pref_vel_x, pref_vel_y, lines_x, lines_y, lines_angles)
	return result

if __name__ == "__main__":
	#Get obstacle angle and velocity in cm/s, add sensors
	#Check steering, read_odom left, right, target_reached
	gpg = EasyGoPiGo3()
	gpg.reset_encoders()
	my_distance_sensor = gpg.init_distance_sensor()
	#gpg.set_robot_constants(wheel_diameter, wheel_base_width)
	target_pose_x = 100.0
	target_pose_y = 0.0
	step  = 5
	pref_vel = 10#cmps_to_dps(30)
	ori = 0.0
	bot_pos_x = 0.0
	bot_pos_y = 0.0
	dist_delta = gpg.read_encoders_average()
	gpg.set_speed(pref_vel)
	gpg.forward()
	at_target = 0
	PI = 3.14
	ori_change = 0.0
	dist_traversed = 0.0
	in_speed = cmps_to_dps(pref_vel)
	gpg.set_speed(in_speed)

	child,parent = multiprocessing.Pipe()
	process = multiprocessing.Process(target=send_data, args=(child,)) 
	process.start()

	while(at_target != 1):
		#Detect Obstacles
		print("***********************************************************************")
		readings = get_readings(my_distance_sensor) #pending
		bot_pos_x = bot_pos_x + (dist_delta * math.cos(ori * PI/180.0))
		bot_pos_y = bot_pos_y + (dist_delta * math.sin(ori * PI/180.0))	
		dist_vec, angle_vec = update_bot_info(readings, ori, bot_pos_x, bot_pos_y)
		#Calculate odom values
		dist_delta = gpg.read_encoders_average() - dist_traversed
		dist_traversed = gpg.read_encoders_average()	
		vel_x, vel_y = read_odom(gpg, ori) 
		parent.send([bot_pos_x, bot_pos_y, vel_x, vel_y])
		#Find new velocity
		result = obstacle(bot_pos_x, bot_pos_y, vel_x, vel_y, target_pose_x, target_pose_y, dist_vec, angle_vec, pref_vel, readings)
		if(abs(result[1]) < 0.1):
			result[1] = 0.0
		if(abs(result[2]) < 0.1):
			result[2] = 0.0
		#Change orientation		
		new_ori = get_rel_ang(bot_pos_x, bot_pos_y, result[1], result[2])
		ori_change = ori - new_ori 
		ori = ori - ori_change
		if(abs(ori_change) > 5.0):
			gpg.set_speed(100)
			speed = ori_change
			if(abs(ori_change) > 180.0):
			    if(ori_change > 0.0):
			        speed = ori_change - 360.0
			    else:
			        speed = ori_change + 360.0
			gpg.turn_degrees(speed)
			gpg.set_speed(in_speed)
		print("Readings= "+str(readings))            
		print("Encoder= "+str(gpg.read_encoders_average()))
		print("Result= "+str(result))        
		print("Bot Pos X= "+str(bot_pos_x)) 
		print("Bot Pos Y= "+str(bot_pos_y))
		print("New Orientation= "+str(new_ori))
		print("Ori Change= "+str(ori_change))
		print("Ori="+str(ori)) 
		print("Speed="+str(result[0]))
		#Change speed
		in_speed = cmps_to_dps(result[0]) #pending
		gpg.set_speed(in_speed) #Degrees per second of the robots wheel
		gpg.forward()
		#Check if target reached
		dist_to_targ = get_dist(bot_pos_x, bot_pos_y, target_pose_x, target_pose_y) 
		print("Dist to Target= "+str(dist_to_targ))
		if(dist_to_targ < 5):
			at_target = 1
	gpg.stop()



