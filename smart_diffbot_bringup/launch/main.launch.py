import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


robot = 'smart_diffbot'


def generate_launch_description():

    ## arguments
    sim = LaunchConfiguration('sim')

    declared_arguments =[]
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Run robot in simulation (sim:=true) or use real hardware (sim:=false)'
        )
    )

    ## Launch simulation
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'simulation.launch.py')]),
        condition=IfCondition(sim),
    )   

    ## Launch description
    return LaunchDescription(declared_arguments + [

        # Launch
        launch_simulation,
        launch_hardware
    ])
    
