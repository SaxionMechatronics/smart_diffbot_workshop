from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    

    ## Controllers

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

        # Nodes
        diff_drive_spawner,
        joint_broad_spawner,

    ])
    
