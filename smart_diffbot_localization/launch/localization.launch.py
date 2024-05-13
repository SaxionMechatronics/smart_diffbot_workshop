import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


robot = 'smart_diffbot'


def generate_launch_description():
    
    ## Parameters
    localization_params = os.path.join(get_package_share_directory(robot+'_localization'), 'config', 'localization_params.yaml')


    ## Nodes

    # Navsat tranform
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        parameters=[localization_params],        
        remappings=[('gps/fix', '/navsat/fix'),
                    ('/imu', '/imu/data'),
                    ('odometry/filtered', '/global_ekf/odometry_filtered'),
                    ('odometry/gps', '/navsat_transform/navsat_odometry'),
                    ('gps/filtered', '/navsat_transform/filtered_fix')
                    ]
    )

    # Global EKF for odom => map tranform 
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf_node',
        parameters=[localization_params],
        remappings=[('odometry/filtered', 'global_ekf/odometry_filtered'),
                    ]
    )
    

    ## Launch description
    return LaunchDescription([
        
        # Nodes
        navsat_transform_node,
        global_ekf_node,

    ])
    
