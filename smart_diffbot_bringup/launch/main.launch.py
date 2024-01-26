import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


robot = 'smart_diffbot'


def generate_launch_description():

    ## Launch simulation
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_bringup'), 'launch', 'simulation.launch.py')]),
    )

    ## Launch control
    launch_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_control'), 'launch', 'control.launch.py')]),
    )

    ## Launch localization
    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot+'_localization'), 'launch', 'localization.launch.py')]),
    )

    ## Launch navigation
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(robot+'_navigation'), 'launch', 'navigation.launch.py')]),
    )


    ## Launch description
    return LaunchDescription([

        # Launch
        launch_simulation,
        launch_control,
        launch_localization,
        launch_navigation,
    ])
    
