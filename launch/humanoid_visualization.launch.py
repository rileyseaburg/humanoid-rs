from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch humanoid robot with RViz visualization"""
    
    # Find package share directory
    pkg_share = FindPackageShare('humanoid_rs')
    
    # Declare launch arguments
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'humanoid.rviz']),
        description='Path to RViz configuration file'
    )
    
    # Include the main humanoid launch file
    humanoid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'humanoid_sim.launch.py'])
        ]),
        launch_arguments={
            'node_name': 'humanoid_viz',
            'namespace': 'humanoid',
        }.items()
    )
    
    # Robot state publisher for URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name="humanoid"><link name="base_link"/></robot>',
            'use_sim_time': True,
        }],
    )
    
    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=None,  # Add condition for use_rviz
    )
    
    return LaunchDescription([
        declare_use_rviz,
        declare_rviz_config,
        humanoid_launch,
        robot_state_publisher,
        rviz_node,
    ])
