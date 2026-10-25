import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('agri_robot')
    tb3_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    world_path = os.path.join(pkg_share, 'worlds', 'field.world')

    return LaunchDescription([
        # 1. تشغيل محاكاة Gazebo مع العالم
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r -v2 ', world_path]}.items()
        ),

        # 2. إدخال الروبوت TurtleBot3 في الحقل
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()
        ),

        # 3. جسر الحركة ROS2 <-> Gazebo لـ cmd_vel و odom
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            output='screen'
        ),

        # 4. تشغيل RViz2 مع ملف الإعدادات
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'agri_config.rviz')],
            output='screen'
        ),
    ])
