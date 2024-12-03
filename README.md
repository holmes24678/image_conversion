# Image Conversion Node
### This repository demonstrates how to convert image into Grayascale and RGB Image by using ROS2 Service call.

### Prerequisites
1. Make sure you have installed ROS2. To install check out https://docs.ros.org/en/humble/index.html
2. make sure you have installed usb_cam package on your machine. if you have not installed check installation guide here https://index.ros.org/p/usb_cam/

### 1. Cloning the repository
```bash
git clone https://github.com/holmes24678/image_conversion_node.git
``` 
### 2. Build the workspace using colcon and source it
```bash
colcon build 
source ./install/setup.bash 
```
### 3. Launch Conversion Node. make sure you have camera harware interface
```bash
ros2 launch image_conversion_node conversion.launch.py
```
### 4. check for ros2 Service list 
```bash
ros2 service list
```
you will find /process_image service is running

### 5. Service call for image conversion
To change the image to Grayascale
```bash
ros2 service call /process_image services/srv/ProcessImage "{mode: grayscale}"
```
To change the image to RGB
```bash
ros2 service call /process_image services/srv/ProcessImage
```

Note : For grayscale 'grayscale'; For color 'default'
