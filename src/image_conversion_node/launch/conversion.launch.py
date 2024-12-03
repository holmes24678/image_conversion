from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ## path to config file for usb_cam 
    cam_parameters_file = "/home/sherlock/cam_pkg/src/usb_cam/config/params_1.yaml"
    ## getting image_conversion_node package
    package_dir = get_package_share_directory('image_conversion_node')
    ## rviz config file
    rviz_config_file = os.path.join(package_dir,'config','display.rviz')

    ##usb_cam_node
    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        output = 'screen',
        arguments=['--ros-args','--params-file', cam_parameters_file],
    )
    ##image_conversion_service_node
    Service_node = Node(
        package="image_conversion_node",
        executable="image_service_node",
        output= "screen",
    )
    ##rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription(
        [
            usb_cam_node,
            Service_node,
            rviz_node,

        ]
    )