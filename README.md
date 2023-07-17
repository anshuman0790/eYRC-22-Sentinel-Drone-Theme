# eYRC-22-Sentinel-Drone-Theme
In this project, the drone was used to canvas the area of land while simultaneously scanning it using GPS in the real world and a ceiling mounted camera. With the help of path planning and pattern matching algorithm, the drone will detect the location of suspicious objects, estimate the coordinate system and send the data back to the main server.

# **Task 0: Sentinel Drone**
**Installation**

This tasks requires you to install Ubuntu 20.04 LTS 446 on your computer. Please prefer a bare metal install (i.e. no virtual machines (VMs)) as misconfiguring your VM instance can lead to all sorts of hard to decipher errors down the line.

Ubuntu 20.04 LTS is a great general purpose Linux distribution that is capable of being daily-driven just fine.

We will be installing ROS 1 Noetic, Geospatial softwares and libraries 1.4k for the future tasks (you can do so now as well if you want), hence given the support for these packages, Ubuntu is the best choice compared to other Linux distributions.

If you face problems during install, seek help from the internet and your peers around you, that’ll be quicker, after that, keep an eye on the QnA with tags task-0 and installation.
Conversely, use the above tags if you want to post asking for help, make sure you have put in effort to solve the problem before asking.

**Learning**

This Task requires you to have or gain a certain comfort with using Python, as this will be the primary programming language used in the Theme.

Check out the relevant page 341, to gain access to Python resource pointers and tips.

**Task**

Installation of Ubuntu 20.04 LTS and relevant softwares and packages on the computer that you will use for solving this theme.

Solving the CodeChef challenge 816 which contains 10 simple Python problem statements, and which will launch in the next week.

# **Task 1A : Building a Control System**
**Task 1A Stabilising the Quadcopter**

**Aim**

The aim of this task is to build a PID control system to stabilise the quadcopter at any given position in a simulation environment in Gazebo


**Prerequisites**

It is presumed that you have successfully completed Task 0 and completed the codechef challenge. Also this task involves writing programs in Python programming language, so get acquainted with the basics of Python and ROS libraries for Pyhton. Follow the given resources to get started with the topics and do not limit yourself with the resources listed, internet is a vast ocean of knowledge, make use of it !


**Installations**

Before proceeding further, you need to install the some softwares and packages. To do that, follow these commands

Create a catkin workspace

cd

mkdir catkin_ws/src -p

cd catkin_ws

catkin init


Build your workspace

cd ~/catkin_ws

catkin build

Each time you build your workspace, you need to source setup.bash file from the catkin_ws/devel folder. Instead of doing this manually, let us add a line in .bashrc

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

Now clone the sentinel drone and other ros packages form GitHub

cd ~/catkin_ws/src 

git clone https://github.com/erts-RnD/sentinel_drone.git --recursive 

git clone https://github.com/simmubhangu/pid_tune.git

**Install dependencies**

sudo apt install ros-noetic-octomap-msgs ros-noetic-octomap-ros ros-noetic-mavlink libgoogle-glog-dev ros-noetic-plotjuggler-ros 
Now build your workspace
cd ~/catkin_ws
catkin build
source ~/.bashrc

**Resources**

1. Python
Python for Beginners - Learn Python in 1 Hour - Programming with Mosh 369

3. ROS
ROS Tutorials - ROS wiki 779
Go through the tutorials till section 1.16

4. Getting started with Quadcopters
Getting started with Quadcopters 844

5. Understanding the edrone model in gazebo simulator
Understanding the edrone model in gazebo 1.0k

6. PID controller
Understanding PID Control, Part 1: What Is PID Control? - MATLAB 689

**Problem Statement**

The task is to build a PID controller for controlling position (x,y,z) of the quadcopter in Gazebo world.

The PID controller will be a closed loop controller with real time position of the quadcopter being fed-back to the controller as a feedback.

The output of the controller will be commands to the quadcopter as angle-setpoints which the quadcopter is supposed to tilt at.

The PID controller will be written as a rosnode written in python programming language

After the PID controller is build and tuned successfully, the quadcopter should be able to move and stabilise at the given setpoint [2,2,20] in the gazebo environment and stay within the error range of ±0.2m in all the coordinates.

Refer to the expected output section to check the output of a PID controller

**Procedure**

Launch the Gazebo world containing the quadcopter and the overhead camera by typing the following command

roslaunch sentinel_drone task_1.launch

Read the boiler-plate pyhton program position_hold.py given in the catkin_ws/src/sentinel_drone/sentinel_drone/scripts folder thoroughly and complete the script to build the PID controller
Tune the PID controller to optimise the performance of the quadcopter and achieve the desired output. After tuning, fix the P, I and D gains in your python script.

Follow the recording and submission instructions to submit your task

# **Task 1B: Geo referencing aerial image using QGIS**

**Task 1B Geo referencing aerial image using QGIS**

**Aim:**

The aim of this task is to understand GIS concepts and getting hands on QGIS software for Geo spatial analysis. In this task you will be generating a georeferenced image using Georeferencer tool in QGIS.


**Resources:**


1. Introduction to GIS 642
2. QGIS software interface 521
3. Importing raster (image) data to QGIS 387
4. Changing Reference system of layers imported 327
5. Using Georeferencer tool in QGIS 356
6. Adding openstreetmap and google satellite image in QGIS 265

**Problem Statement:**

The task is for getting familiar with QGIS software for basic analysis that will be needed in this theme.

Download the satellite image 575 and the aerial image 505.

 Note: Satellite images are very large in size and covers a wide area. Provided satellite image is a high resolution satellite image with spatial resolution of 30cm(i.e. 1 pixel represents 30cm in ground)

Open the satellite image in QGIS software.

Select EPSG4326 referencing system.

Open the georeferencer tool in QGIS. Add the aerial image to to be georeferenced.

Mark proper GCPs of the both actual image and reference by zooming and matching features.

Create and save the georeferenced image created by using georeferencer tool.


# **Task 2A: Waypoint Navigation**

**Task 2A Flying the Quadcopter through Way Points**

**Aim**
The aim of this task is to write a wrapper over the existing PID control system, written in Task 1 to fly the quadcopter through a list of set points in the simulation environment in Gazebo.

**Prerequisites**
It is presumed that you have successfully completed Task 1 and the prerequisites of the same. No new resources, apart from task 1 are needed for this task, however do not limit yourself with the same, the internet is a vast ocean of knowledge, make use of it!

**Installations**
We’ve pushed new code to the exisiting package. You simply need to pull it and run catkin build

cd ~/catkin_ws/src/sentinel_drone

git pull

catkin build

**Problem Statement**

The quadcopter should move through each set point in the gazebo environment

-Takeoff
- [0,0,23]
- [2,0,23]
- [2,2,23]
- [-2,2,23]
- [-2,-2,23]
- [2,-2,23]
- [2,0,23]
- [0,0,23]

A waypoint will be considered success if the drone gets within the error range of ±0.2m in all the coordinate axis for even one instance

**Procedure**

Launch the Gazebo world containing the quadcopter and the overhead camera by typing the following command

roslaunch sentinel_drone task_2.launch

Make a new pyhton script waypoint_navigation.py in the catkin_ws/src/sentinel_drone/sentinel_drone/scripts folder and complete the script to fly the drone through the mentioned set points.

Potentially use the PID values from task 1 to achieve optimal performance of the quadcopter flight in your python script.

Follow the recording and submission instructions to submit your task

# **Task 2B: Detect colored object from the given image and find its pixel co-ordinates**

**Aim:**
The aim of this task is to detect object from the given image and find its pixel co-ordinates.

**Resources:**

Basics of opencv 438

 For more detail description you can refer opencv official documentation 239.
 
**Problem Statement:**

In this task you will use computer vision techniques to find the pixel co-ordinates of the yellow block kept on the arena.


Download the image 530 named yellow_detect.jpeg. You will see a yellow block kept on the satellite image.

Use image processing technique to detect the yellow colored block kept on the arena.

Find the pixel co-ordinate of the center of the detected block.

 Hint: Use image thresholding technique and then use proper filtering to extract only the block from the satellite image.
You are free to use any other approach.

# **Task 2C: Detecting suspecious objects using drone**

**Aim:**
The aim of this task is to scan the entire city area(arena) using drone, detect object from the drone camera.

**Prerequisites**
**Installations**
Update the sentinel_drone ros package from github


cd ~/catkin_ws/src/sentinel_drone

git pull origin main

cd ~/catkin_ws

catkin build

source ~/catkin_ws/devel/setup.bash

**Resources**
Basics of opencv 105

 For more detail description you can refer opencv official documentation 42.
 
Reading images from drone camera to process using open cv

Working With ROS and OpenCV in ROS Noetic – Automatic Addison 294

OpenCV with ROS using Python | turtlebot2-tutorials 244

**Problem Statement:**

In this task you will use computer vision techniques to find the location of the image taken from the drone where the object is detected.


In task 2B you successfully detected the object and its pixel location. Now in this task you need to extend this algorithm to search for the object on the arena using drone.


 Hint: Make your own approach to plan the search over the entire arena, and then hover over the yellow block to maintain the block in the centre of the image frame
 
 Note: All the script should run at the start of the run, You are not allowed to run any script in between the run.
 
**Procedure:**

Launch sentinel drone in Gazebo simulator.

roslaunch sentinel_drone task_2c.launch

Drone will fly and scan the city.

Detect the suspicious activities indicated by yellow block on the arena using the camera attached on the drone.

Once yellow block is detected, you need to align the drone in a way that the yellow block is in the centre of the drone camera frame seen from the drone.

Hover over the location of the block, the height of the drone is to be chosen by you (Remember, more higher you go, more area can be seen by the drone camera, but at the same time the accuracy of the block decreases with height as well as the overhead camera has a limited field of view, so the WhyCon marker also has to be in frame of the overhead camera. Optimise your drone height wisely)

 WARNING: Do NOT hard code the location of yellow block instead write a robust search and scanning algorithm, this task has only one block, later tasks will have multiple blocks at random positions. If hard coding is found, no marks will be given

# **Task 2D: Finding geolocation of the suspecious objects using drone**

**Task 2D: Finding geolocation of the suspecious objects using drone**

**Aim:**
The aim of this task is to geolocate (latitude and longitude) suspicious object on the map.

**Resources:**

Basics of opencv 74

For more detail description you can refer opencv official documentation 32.

GDAL for geoprocessing 440

Plotting location of object detected in qgis using python script. 347

**Installations**

Update the sentinel_drone ros package from github

cd ~/catkin_ws/src/sentinel_drone

git pull origin main

cd ~/catkin_ws

catkin build

source ~/catkin_ws/devel/setup.bash

**Problem Statement:**
In this task you will use computer vision techniques to find the location (latitude and longitude) of the image taken from the drone where the object is detected.

In task 2B and task 2C you successfully detected the object and its pixel location. Now in this task you need to get the geo-location (longitude and latitude) of the object on the map.

 Hint: One approach could be taking a photo of the yellow block after finding it (Task 2C) and georeference the image with satellite image 214 (this a lower resolution of the satellite image) to get the geo-location of the centroid pixel co-ordinates of the object.
 
In task 1B you used QGIS georeferencer tool to manually find the matching features(GCPs- Ground Control Points) and georeference the aerial image but in this task you need to use image processing techniques to find the matched features in both aerial and satellite image and then use GDAL library to georeference the aerial image.


There are various method of feature matching like SIFT, SURF, ORB etc for matching features.

You are also free to use any other approach to find the exact location(latitude and longitude) of an object.

After getting the locations of the objects you need to plot them in QGIS map canvas on an openstreet map layer of a given arena using a python script.


 Note: All the script should run at the start of the run, You are not allowed to run any script in between the run.
 
**Procedure:**

QGIS application should be opened with Openstreet map as base layer containing the location of the arena. ROS node should be running in QGIS python console which will suscribe topic “geolocation” with message type Geolocation for reading the location of point passed by a rostopic from drone image.
 geolocation topic must use a custom message type containing objectid, latitude and longitude. The objectid should be in the following format: obj0, obj1 and obj3 for detected object number 1,2 and 3 respectively. Latitude and Longitude have the float data type.
 
Geolocation Message type:

saail@eyantra:~/catkin_ws$ rosmsg show Geolocation

[sentinel_drone/Geolocation]:

string objectid

float32 lat

float32 long


Launch sentinel drone in Gazebo simulator using terminal.


roslaunch sentinel_drone task_2d.launch

Sentinal Drone will fly and scan the city.

Detect the suspicious activities indicated by yellow block on the arena using the camera attached on the drone.


Once yellow block is detected the geo location need to be found.


 Hint: Georeferencing the frame where object is detected.
 
Geolocation point needs to be passed to a rostopic named “geolocation”(having message type Geolocation) through rosnode for plotting it on a base map of Openstreet map.


All the points should be plotted on QGIS base layer map wherever the object was detected.


