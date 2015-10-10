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

4. Edit the camera.launch argument for the correct video input.
  * $ `cd ~/catkin_ws/src/usma_remote_interface/launch`
  * $ `ls /dev/video*` (to see availbale video devices)
  * $ `vim camera.launch` (or your preferred editor)
  * Change the video device. Internal webcams will typically be video0, an external webcam could be video1.
  * `<arg name="video_device" default="/dev/video1" />`

5. Set the IP address for the webserver.

6. Launch camera.launch.
  * $ `roslaunch usma_remote_interface camera.launch`

7. Check to make sure your video feed is working.  Change the image topic to match the topic name.
  * $ `rosrun image_view image_view image:=/usb_cam/image_raw`

8. Go to a web browser and enter in the IP address for the computer.
  * url:   \<ip_address_server\>?ip=\<ip_address_ros\>
  * example:   localhost?ip=localhost
