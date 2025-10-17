from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

robot = 'smart_diffbot'

def generate_launch_description():
    ## Arguments
    sim_arg = DeclareLaunchArgument(name='sim', default_value='true', choices=['true', 'false'],
                                    description='Set to true to switch from hardware to simulation in the loop')
    ## Parameters
    controller_params = os.path.join(get_package_share_directory(robot+'_control'), 'config', 'controller_params.yaml')

     ## Controller manager (only on real robot, Gazebo start one by default)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim')),
        emulate_tty=True,
    )

    ## Controllers

    # Spawn position and velocity controllers
    pos_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["camera_controller"],
    )

    vel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--inactive"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # Spawn diff drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # Spawn joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

 
    ## Launch description
    return LaunchDescription([
        # Arguments
        sim_arg,

        # Nodes
        ros2_control_node,
        pos_controller,
        vel_controller,
        diff_drive_spawner,
        joint_broad_spawner,

    ])
    
