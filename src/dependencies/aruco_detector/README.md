The Dnt package to run Motion Capture for multiple bots using aruco markers  

To run package:

1) In your ros_ws/src
```bash
git clone https://github.com/DNT-Dev/auv_mocap.git 
cd ..
catkin_make
```

3) Have camera feeds outputing to rostopics:  
  /camera_1/image_raw  
  /camera_2/image_raw  
  /camera_3/image_raw  
  ... etc  

  Put the calibration files for the same in ${ROS_WORKSPACE}/src/auv_mocap/calibration_files and name them camera_%d.txt like camera_1.txt, camera_2.txt and so on

4) Run Aruco marker detection node
   
  ```rosrun auv_mocap detector_node _num_cameras:=${NUMBER OF CAMERAS}```   

4) Run the transform calculator node   

  ```rosrun auv_mocap tf_node```
