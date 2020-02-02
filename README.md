# kalman_filter_mg_cs169
This is Kalman filter package for CS 69 / 169 class at Dartmouth Robotics perception course.
Wors are coded by Mingi Jeong, 1st year Ph.D. Students in Robotics/Computer Science at Dartmouth. This program was conducted on ROS kinetic and Linux Ubuntu 16.04

I am pleased to take this class taught by Prof.Alberto Quattrini Li.

# How to download and install necessary packages

1. git clone to your catkin_ws folder
2. Make sure you download rosbag file recorded from Husarion Rosbot 2.0 from the following link
   https://drive.google.com/open?id=1onVc88q--nnUFm7Kv0Schqn1i-yoMVre
3. Edit launch files rosbag play node as per path you will call the downloaded bag files
    e.g. (args="--clock /home/mingi/catkin_ws/src/kalman_filter_mg_cs169/bags/BAGFILENAME.bag")
4. Edit file save path (csv) inside python scripts (cmd_estimate.py / cmd_estimate_camera.py / pose_estimate.py / pose_estimate_camera.py)
5. Edit path in plotting script (plotter.py)
6. source ~/catkin_ws/devel/setup.bash (afterwards recommended : rospack profile)
7. run command to make your python scripts executable e.g (chmod +x ~/catkin_ws/src/kalman_filter_mg_cs169/scripts/initial_pose.py)

# How to execute

For each pose estimate based on subscribed topics (e.g. cmd_vel and scan), a launch file is made accordingly
in order to execute main python scripts connected to it. You should follow this procedure by making sure that
initial pose can be published into different nodes (from different launch files) in a same way.

1. screen -R core (roscore)
2. screen -R initializer (rosrun kalman_filter_mg_cs169 initial_pose.py)
This initializer is essential so that third step can be done.

3. execute a necessary launch file according to topics you want. (roslaunch kalman_filter_mg_cs169 cmd_estimate.launch)
* For your reference, task 1 can be performed by "cmd_estimate.launch". However, publishing PoseWithCovariance messages and
tf transformation can be confirmed on other launch files (Only difference is that the same message and topic is based on which data it
is calculated from.)
* Task 2 - A and 2 - C can be done by "roslaunch kalman_filter_mg_cs169 pose_estimate.launch"
Task 2 - B and 2 - F can be done by "roslaunch kalman_filter_mg_cs169 cmd_estimate.launch"
Task 2 - D can be done by "roslaunch kalman_filter_mg_cs169 cmd_estimate_camera.launch"
Task 2 - E can be done by "roslaunch kalman_filter_mg_cs169 pose_estimate_camera.launch"
* Task 3 as plotting can be performed on command line "python /home/mingi/catkin_ws/src/kalman_filter_mg_cs169/scripts/plotter.py"
* plotter_ros.py (plotting through rosrun) was not executable on my laptop due to the version conflict, but I contained it here for the reference.
