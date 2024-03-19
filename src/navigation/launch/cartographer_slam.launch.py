import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_file_dir = os.path.join(get_package_share_directory('navigation'), 'launch')
    pkg_cartographer = get_package_share_directory('fws_cartographer')
    pkg_fws_control = get_package_share_directory('fws_control')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fws_control, 'launch', 'fws_control.launch.py'),
            )
        ), 

        Node(
        package = "joy_linux",
        executable = "joy_linux_node"
        ),

        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'front_lrf_link',
                'inverted': False,
                'angle_compensate': True,
                'topic_name': 'scan',
            }],
        ),
            
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan',
                'odom_topic' : '/odom',
                'publish_tf' : True,
                'base_frame_id' : 'base_footprint',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : '',
                'freq' : 14.0}],
            ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cartographer, 'launch', 'cartographer.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_cartographer, 'launch', 'occupancy_grid.launch.py')
            ),
        ),
    ])
