import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


robot = 'smart_diffbot'
world = 'docking_world'


def generate_launch_description():
    
    
    ## Robot model
    xacro_file = os.path.join(get_package_share_directory(robot+'_description'), 'model', 'urdf', 'robot.urdf.xacro')

    ## Gazebo world 
    world_file = os.path.join(get_package_share_directory(robot+'_description'), 'model', 'world', world+'.sdf')


    ## Launch files

    # Start Gazebo 
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments=[('gz_args', [' -r -v 0 ' + world_file])],
    )

    ## Nodes

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim:=True']), value_type=str)
            }],
        emulate_tty=True
    )

    # Spawn Gazebo model
    spawn_sim_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'smart_diffbot', '-topic', 'robot_description', '-z', '1.0'],
        output='screen',
    )

    # Bridge topics from Gazebo to ROS2
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                   '/navsat/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
                   '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        output='screen',
    )


    ## Launch description
    return LaunchDescription([

        # Launch
        gazebo,

        # Nodes
        robot_state_publisher,
        spawn_sim_robot,
        gz_bridge_node,

    ])
    
