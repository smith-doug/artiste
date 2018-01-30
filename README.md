artiste

Usage:  
1. Publish a transform between /world and /camera frame   
```rosrun tf static_transform_publisher 0.22 0.20 0.40 -1.57 0.0 -0.4 /world /camera_frame 10```
2. Publish an image topic   
```roslaunch opencv_testing image.launch file:=~/ws/src/opencv_testing/image/keba-square.png```
3. Run the artiste node  
```rosrun artiste artiste_node```   
4. Publish an empty message to /start_cart_move  
```rostopic pub -1 /start_cart_move std_msgs/Empty "{}"```
