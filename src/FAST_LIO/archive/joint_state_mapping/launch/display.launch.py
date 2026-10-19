import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_share = get_package_share_directory('mujoco_simulator')

    urdf_path = os.path.join(
        description_share,
        'assets',
        'resource',
        'robot',
        'pm_v2',
        'urdf',
        'serial_pm_v2.urdf',
    )
    
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_desc, 
                    'use_sim_time': True
                }]
        ),

        Node(
            package='joint_state_mapping',
            executable='joint_state_mapper',
            name='joint_state_mapper',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])


    