import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


robot = 'smart_diffbot'
global_costmap = 'empty'


def generate_launch_description():
    
    ## Parameters
    navigation_params = os.path.join(get_package_share_directory(robot+'_navigation'), 'config', 'navigation_params.yaml')

    ## Behavior trees
    navigate_to_pose_bt = os.path.join(get_package_share_directory(robot+'_navigation'), 'behavior_trees', 'navigate_to_pose_w_simple_recovery.xml')
    navigate_through_poses_bt = os.path.join(get_package_share_directory(robot+'_navigation'), 'behavior_trees', 'navigate_through_poses_w_simple_recovery.xml')

    ## Global costmap
    costmap_file = os.path.join(get_package_share_directory(robot+'_navigation'), 'costmaps', global_costmap+'.yaml')


    ## Nodes

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        emulate_tty=True, 
        parameters=[{'yaml_filename': costmap_file,}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        emulate_tty=True, 
        parameters=[navigation_params],
    )
    
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        emulate_tty=True, 
        parameters=[navigation_params],
    )
            
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        emulate_tty=True, 
        parameters=[navigation_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        emulate_tty=True, 
        parameters=[navigation_params],
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        emulate_tty=True, 
        parameters=[{'default_nav_to_pose_bt_xml': navigate_to_pose_bt}, 
                    {'default_nav_through_poses_bt_xml': navigate_through_poses_bt},
                    navigation_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        emulate_tty=True, 
        parameters=[navigation_params],
        remappings=[('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel_unstamped')],
    )    

    ## Lifecycle manager
    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'velocity_smoother']

    lifecycle_manager =  Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        emulate_tty=True, 
        parameters=[{'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )
  

    ## Launch description
    return LaunchDescription([

        # Nodes
        map_server,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother, 
        lifecycle_manager,

    ])