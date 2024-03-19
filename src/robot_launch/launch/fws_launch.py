#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_fws_control = get_package_share_directory('fws_control')

    joy_node = Node(
        package = "joy_linux",
        executable = "joy_linux_node"
    )   

    spawn_robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fws_control, 'launch', 'fws_control.launch.py'),
        )
    )  

    return LaunchDescription([
        joy_node,
        spawn_robot_control,
    ])