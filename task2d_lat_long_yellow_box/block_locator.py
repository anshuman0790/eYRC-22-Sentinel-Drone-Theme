#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from std_msgs.msg import String
import rospy
import time
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import shutil
from osgeo import gdal
import osr
from sentinel_drone.msg import Geolocation
import subprocess


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,24] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		# whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500 
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp=[0,0,0]
		# self.Ki=[0,0,0]
		# self.Kd=[0,0,0]
		self.Kp = [57.66,54.66,54.36]
		self.Ki = [0,0,0.144]
		self.Kd = [1031.2,1029,392]


		#-----------------------Add other required variables for pid here ----------------------------------------------
		
		self.roll_prev_error = 0                                                                            # for x_axis
		self.pitch_prev_error = 0                                                                           # for y_axis
		self.throttle_prev_error = 0                                                                        # for z_axis 
		
		self.roll_error = 0                                                                                 # for x_axis
		self.pitch_error = 0                                                                                # for y_axis
		self.throttle_error = 0                                                                             # for z_axis 
		
		self.roll_sum_error = 0                                                                             # for x_axis
		self.pitch_sum_error = 0                                                                            # for y_axis
		self.throttle_sum_error = 0                                                                         # for z_axis 
		
		#self.prev_time = [0,0,0]     
		
		self.min_roll = 1200 
		self.max_roll = 1800   
		self.min_pitch = 1200 
		self.max_pitch = 1800
		self.min_throttle = 1325 
		self.max_throttle = 1745                                                                  
		
		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]	
		
		self.max_error_roll = 0.1
		self.min_error_roll = -0.1
		self.max_error_pitch = 0.1
		self.min_error_pitch = -0.1
		self.max_error_throttle = 0.2
		self.min_error_throttle = -0.2

		self.sample_time = 0.050
		self.x = [0.0,0.0,0.0]
		self.y = [0.0,0.0,0.0]
		self.geo = Geolocation()
		

		
		self.speed=[0.0,0.0,0.0]
		self.px1 = [0.0,0.0,0.0]
		self.py1 = [0.0,0.0,0.0]
		#-----------------------Add other required variables for pid here ----------------------------------------------
        








		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.throttle_error_pub = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error',Float64,queue_size=1)

		# Publishing /geolocation0, /geolocation1, /geolocation2
		self.obj0 = rospy.Publisher('geolocation1', Geolocation, queue_size=1)
		self.obj1 = rospy.Publisher('geolocation2', Geolocation, queue_size=1)
		self.obj2 = rospy.Publisher('geolocation3', Geolocation,queue_size=1)







		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/whycon_display/image_view/output', Image , self.image)
		rospy.Subscriber('/whycon_display/image_view/output', Image , self.georeference)
		#rospy.Subscriber('set_point',Pose,self.set_point)


		#-------------------------Add other ROS Subscribers here----------------------------------------------------
        





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
	
	



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        # Callback function for /pid_tuning_altitude
# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly		
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06 
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06 
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3

		
		#---------------------------------------------------------------------------------------------------------------



	
    
	def image(self,data):
		br = CvBridge()
 
        # Output debugging information to the terminal
   
        # Convert ROS Image message to OpenCV image
		current_frame = data
		self.img = current_frame
        #img = cv.imread("yellow_detect.jpeg")
        
		image = br.imgmsg_to_cv2(self.img)
		image = cv2.GaussianBlur(image, (27, 27), 0)
		
		original = br.imgmsg_to_cv2(self.img)
		image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		left = np.array([22, 93, 0], dtype="uint8")
		right = np.array([45, 255, 255], dtype="uint8")
		mask = cv2.inRange(image, left, right)
		
		contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = contours[0] if len(contours) == 2 else contours[1]
		
		i = 0
		x = [0] * len(contours)
		y = [0] * len(contours)
		w = [0] * len(contours)
		h = [0] * len(contours)
		for c in range(len(contours)):
			x[c], y[c], w[c], h[c] = cv2.boundingRect(contours[c])			
			i = i + 1
		index = 0
		a = 0
		c = 0
		n = 0
		for f in contours:
			t = cv2.minAreaRect(contours[n])
			self.x[n] = t[0][0]
			self.y[n] = t[0][1]
			n = n + 1
		

	def georeference(self,data):
		# Read the query image as query_img and train image This query image is what you need to find in train image Save it in the same directory with the name image.jpg 
		br = CvBridge() 
        # Output debugging information to the terminal
   
        # Convert ROS Image message to OpenCV image
		current_frame = data
		img = current_frame        
        
		image = br.imgmsg_to_cv2(img)
		cv2.imwrite('test.jpg',image)
		query_img = image
		train_img = cv2.imread('/home/anshuman/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/task2d.tif')
		
  
        # Convert it to grayscale         
		query_img_bw = cv2.cvtColor(query_img,cv2.COLOR_BGR2GRAY)
		train_img_bw = cv2.cvtColor(train_img,cv2.COLOR_BGR2GRAY)
  
        # Initialize the ORB detector algorithm
		orb = cv2.ORB_create(nfeatures=2000)
         
        # Now detect the keypoints and compute the descriptors for the query image and train image
		queryKeypoints, queryDescriptors = orb.detectAndCompute(query_img_bw,None)
		trainKeypoints, trainDescriptors = orb.detectAndCompute(train_img_bw,None)
        
        # Initialize the Matcher for matching the keypoints and then match the keypoints
		bf = cv2.BFMatcher()
		matches = bf.knnMatch(queryDescriptors,trainDescriptors,k=2)
        
        # draw the matches to the final image containing both the images the drawMatches() function takes both images and keypoints and outputs the matched query image with its train image
		good=[]
		for m,n in matches:
			if m.distance < 0.43*n.distance:
				good.append(m)
		final_img = cv2.drawMatchesKnn(query_img, queryKeypoints,
        train_img, trainKeypoints, [good],None,flags=2)
        
		final_img = cv2.resize(final_img, (1600,850))
		#print(good[0])
        
        # Initialize lists
		list_kp1 = list()
		list_kp2 = list()

        # For each match...
		for mat in good:

            # Get the matching keypoints for each of the images
			img1_idx = mat.queryIdx
			img2_idx = mat.trainIdx

            # x - columns
            # y - rows
            # Get the coordinates
			(x1, y1) = queryKeypoints[img1_idx].pt
			(x2, y2) = trainKeypoints[img2_idx].pt

            # Append to each list
			list_kp1.append((x1, y1))
			list_kp2.append((x2, y2))
		
		tif = gdal.Open('/home/anshuman/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/task2d.tif')
		gt = tif.GetGeoTransform()
		x_min = gt[0]
		x_size = gt[1]
		y_min = gt[3]
		y_size = gt[5]
		
		i = 0
        
		list_img=list()
		
		for x,y in list_kp2:
			px = x * x_size + x_min #x pixel
			py = y * y_size + y_min #y pixel
			list_img.append((px,py))           
			i=i+1
        
		orig_fn='test.jpg'
		output_fn='output.tif'
		ds=gdal.Open('test.jpg')
		I = ds.ReadAsArray(0,0,ds.RasterXSize,ds.RasterYSize)
		out=gdal.GetDriverByName('GTiff')
		sr = osr.SpatialReference()
		sr.ImportFromEPSG(4326)
		out = out.Create(output_fn,ds.RasterXSize,ds.RasterYSize,I.shape[0])
		for nb_band in range(I.shape[0]):
			out.GetRasterBand(nb_band+1).WriteArray(I[nb_band,:,:])

		gcps = list()
		i=0
		for n in list_img:
			gcps.append(gdal.GCP(n[0],n[1],0,list_kp1[i][0],list_kp1[i][1]))
			i=i+1
		out.SetProjection(sr.ExportToWkt())
		wkt=out.GetProjection()
		out.SetGCPs(gcps, wkt)
		out=None
		from_SRS = "EPSG:4326"
		to_SRS = "EPSG:4326"

		src='output.tif'
		dest= 'modified.tif'

		cmd_list = ["gdalwarp","-r", "bilinear", "-s_srs", from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]

		subprocess.run(cmd_list)

		geo = gdal.Open('modified.tif')
		ft = geo.GetGeoTransform()

		x1_min = ft[0]
		x1_size = ft[1]
		y1_min = ft[3]
		y1_size = ft[5]
		
		
		for ed in range(3):
			self.px1[ed] = self.x[ed] * x1_size + x_min - 0.000211
			self.py1[ed] = self.y[ed] * y1_size + y1_min
		print(self.px1,self.py1)
		

	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum	
		self.roll_error = self.drone_position[0] - self.setpoint[0]                                                       #error[0] corresponds to error in x...
		self.pitch_error = self.drone_position[1] - self.setpoint[1]                                                      #error[1] corresponds to error in y...
		self.throttle_error= self.drone_position[2] - self.setpoint[2]                                                    #error[2] corresponds to error in z...
			
		self.cmd.rcRoll = int(1500 - (self.Kp[0] * self.roll_error + self.Kd[0] * (self.roll_error - self.roll_prev_error) + self.Ki[0] * self.roll_sum_error))
		self.cmd.rcPitch = int(1500 + (self.Kp[1] * self.pitch_error + self.Kd[1] * (self.pitch_error - self.pitch_prev_error) + self.Ki[1] * self.pitch_sum_error))
		self.cmd.rcThrottle = int (1500 + (self.throttle_error * self.Kp[2] + (self.throttle_error - self.throttle_prev_error) * self.Kd[2] + self.throttle_sum_error * self.Ki[2]))
	
		if self.cmd.rcRoll > self.max_roll:
			self.cmd.rcRoll = self.max_roll
		if self.cmd.rcRoll < self.min_roll:
			self.cmd.rcRoll = self.min_roll
	
		if self.cmd.rcPitch > self.max_pitch:
			self.cmd.rcPitch = self.max_pitch
		if self.cmd.rcPitch < self.min_pitch:
			self.cmd.rcPitch = self.min_pitch
					
		if self.cmd.rcThrottle > self.max_throttle:
			self.cmd.rcThrottle = self.max_throttle
		if self.cmd.rcThrottle < self.min_throttle:
			self.cmd.rcThrottle = self.min_throttle
		
					#anti-windup		
		if self.roll_sum_error > self.max_error_roll:
			self.roll_sum_error = self.max_error_roll
		if self.roll_sum_error < self.min_error_roll:
			self.roll_sum_error = self.min_error_roll
	
		if self.pitch_sum_error > self.max_error_pitch:
			self.pitch_sum_error = self.max_error_pitch
		if self.pitch_sum_error < self.min_error_pitch:
			self.pitch_sum_error = self.min_error_pitch
					
		if self.throttle_sum_error > self.max_error_throttle:
			self.throttle_sum_error = self.max_error_throttle
		if self.throttle_sum_error < self.min_error_throttle:
			self.throttle_sum_error = self.min_error_throttle
			
			
		self.roll_prev_error = self.roll_error
		self.pitch_prev_error = self.pitch_error 
		self.throttle_prev_error = self.throttle_error
		
		self.roll_sum_error = self.roll_sum_error + self.roll_error                                                       #s_error[0] (sum of error) corresponds to error in x...  
		self.pitch_sum_error = self.pitch_sum_error + self.pitch_error                                                    #s_error[1] (sum of error) corresponds to error in y...  
		self.throttle_sum_error = self.throttle_sum_error + self.throttle_error                                           #s_error[2] (sum of error) corresponds to error in z... 	

		
		
						
		# #------------------------------------------------------------------------------------------------------------------------
		
		self.command_pub.publish(self.cmd)
		self.roll_error_pub.publish(self.roll_error)
		self.pitch_error_pub.publish(self.pitch_error)
		self.throttle_error_pub.publish(self.throttle_error)
		self.geo.objectid = 'object0'
		self.geo.lat = self.px1[0]
		self.geo.long = self.py1[0]
		self.obj0.publish(self.geo)
		self.geo.objectid = 'object1'
		self.geo.lat = self.px1[1]
		self.geo.long = self.py1[1]
		self.obj1.publish(self.geo)
		self.geo.objectid = 'object2'
		self.geo.lat = self.px1[2]
		self.geo.long = self.py1[2]
		self.obj2.publish(self.geo)




if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		e_drone.pid()
		r.sleep()