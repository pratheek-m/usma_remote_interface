usma_remote_interface
=====================
1. Install all dependencies (assuming ROS Indigo).
 * $ `sudo apt-get install ros-indigo-rosbridge-server`
 * $ `sudo apt-get install ros-indigo-mjpeg-server`
 * $ `sudo apt-get install ros-indigo-usb-cam`
 
2. Clone the usma_remote_interface ros package into your catkin workspace src folder.
  * $ `cd ~/catkin_ws/src`
  * $ `git clone https://github.com/westpoint-robotics/usma_remote_interface.git usma_remote_interface`
  
3. Run the setup script.
  * $ `cd ~/catkin_ws/src/usma_remote_interface/scripts`
  * $ `./usma_remote_interface_setup.sh`

4. Edit the master.launch argument for the correct video input.
  * $ `cd ~/catkin_ws/src/usma_remote_interface/launch`
  * $ `ls /dev/video*` (to see available video devices)
  * $ `vim master.launch` (or your preferred editor)
  * Change the video device. Internal webcams will typically be video0, an external webcam could be video1.
  * `<arg name="video_device" default="/dev/video1" />`

5. Edit the javascript config.
  * $ `roscd usma_remote_interface/webpage`
  * $ `vim config.js` (or your preferred editor)
  * Edit the file to reflect desired topics and IPs

6. Launch master.launch.
  * $ `roslaunch usma_remote_interface master.launch`

7. Check to make sure your video feed is working.  Change the image topic to match the topic name.
  * $ `rosrun image_view image_view image:=/usb_cam/image_raw`

8. Go to a web browser and enter in the IP address for the computer.
  * url:   \<ip_address_server\>
  * example:   localhost

9. To set up a ROS master to connect multiple machines over the network:
  * $ `cd ~`
  * $ `vim .bashrc` (or your preferred editor)
  * Add this export to the end of your .bashrc file that matches the IP of your master machine (i.e. on the robot):
      * export ROS_MASTER_URI=http://ros02:11311
  * Both the client and server should have this export in their .bashrc
  * Edit your /etc/hosts to include the computer name and IP address for the other computer similar to these   instructions: http://www.faqs.org/docs/securing/chap9sec95.html
  * Other references:
    * http://wiki.ros.org/ROS/Tutorials/MultipleMachines
    * http://wiki.ros.org/ROS/NetworkSetup
  
10. To run teleop from your client machien, plug-in your joystick and execute:
  * $ `roslaunch teleop_twist_joy teleop.launch`
  * You may need to remap twist message names
