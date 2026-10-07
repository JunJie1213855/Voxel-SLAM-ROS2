import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import launch.events
from launch.actions import EmitEvent
from launch.events.process import ProcessExited
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('voxel_slam')
    rviz_config_file = os.path.join(pkg_dir, 'rviz_cfg', 'back.rviz')
    config_file = os.path.join(pkg_dir, 'config', 'robosense.yaml')

    lid_topic_arg = DeclareLaunchArgument('lid_topic', default_value='/sensing/lidar/corrected/front_left/points_cropped', description='Lidar topic')
    bag_path_arg = DeclareLaunchArgument('bag_path', default_value='/home/pix/code/mapping_ws/src/Voxel-SLAM/rosbag/rosbag2_2026_04_20-14_34_07', description='Path to the ROS bag')
    bagname_arg = DeclareLaunchArgument('bagname', default_value='test', description='Name of the bag for saving outputs')

    voxelslam_node = Node(
        package='voxel_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        parameters=[
            config_file,
            {
                'finish': False, 
                'use_sim_time': True,
                'General.lid_topic': LaunchConfiguration('lid_topic'),
                'General.bag_path': LaunchConfiguration('bag_path'),
                'General.bagname': LaunchConfiguration('bagname')
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        lid_topic_arg,
        bag_path_arg,
        bagname_arg,
        
        voxelslam_node,
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=voxelslam_node,
                on_exit=[
                    launch.actions.LogInfo(msg="SLAM finished cleanly. Shutting down complete launch..."),
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ]
            )
        )
    ])
