# humanoid_sim.launch.py
# ROS2 launch file for humanoid robot in simulation

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for humanoid robot simulation."""
    
    # Declare launch arguments
    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='humanoid_sim',
        description='Name of the ROS2 node'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/humanoid',
        description='Namespace for the robot'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Clock bridge node (publishes /clock for simulation time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=LaunchConfigurationEquals('use_sim_time', 'true'),
    )
    
    # Humanoid ROS node with simulation clock
    humanoid_node = Node(
        package='humanoid_rs',
        executable='humanoid_ros_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'clock_mode': 'Simulation',
            'joint_state_rate': 100.0,
            'imu_rate': 200.0,
            'pose_rate': 100.0,
            'real_time_factor': 1.0,
        }],
        output='screen',
    )
    
    # Robot state publisher for TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', PathJoinSubstitution([
                    FindPackageShare('humanoid_rs'),
                    'urdf',
                    'humanoid.urdf.xacro'
                ])]),
                value_type=str
            ),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=LaunchConfiguration('namespace'),
        condition=LaunchConfigurationEquals('gui', 'true'),
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('humanoid_rs'),
            'config',
            'humanoid.rviz'
        ])],
        condition=LaunchConfigurationEquals('rviz', 'true'),
    )
    
    # Log info
    log_info = LogInfo(
        msg=[
            'Starting humanoid simulation:\n',
            '  Node: ', LaunchConfiguration('node_name'), '\n',
            '  Sim time: ', LaunchConfiguration('use_sim_time'),
        ]
    )
    
    return LaunchDescription([
        declare_node_name,
        declare_namespace,
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        log_info,
        clock_bridge,
        humanoid_node,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
