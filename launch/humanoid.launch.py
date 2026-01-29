# humanoid.launch.py
# ROS2 Python launch file for humanoid robot

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for humanoid robot."""
    
    # Declare launch arguments
    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='humanoid_ros_node',
        description='Name of the ROS2 node'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/humanoid',
        description='Namespace for the robot'
    )
    
    declare_clock_mode = DeclareLaunchArgument(
        'clock_mode',
        default_value='WallTime',
        description='Clock mode: WallTime, Simulation, or External'
    )
    
    declare_joint_state_rate = DeclareLaunchArgument(
        'joint_state_rate',
        default_value='100.0',
        description='Joint state publication rate in Hz'
    )
    
    declare_imu_rate = DeclareLaunchArgument(
        'imu_rate',
        default_value='200.0',
        description='IMU publication rate in Hz'
    )
    
    declare_pose_rate = DeclareLaunchArgument(
        'pose_rate',
        default_value='100.0',
        description='Pose publication rate in Hz'
    )
    
    declare_real_time_factor = DeclareLaunchArgument(
        'real_time_factor',
        default_value='1.0',
        description='Real-time factor for simulation'
    )
    
    # Create the humanoid node
    humanoid_node = Node(
        package='humanoid_rs',
        executable='humanoid_ros_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'clock_mode': LaunchConfiguration('clock_mode'),
            'joint_state_rate': LaunchConfiguration('joint_state_rate'),
            'imu_rate': LaunchConfiguration('imu_rate'),
            'pose_rate': LaunchConfiguration('pose_rate'),
            'real_time_factor': LaunchConfiguration('real_time_factor'),
        }],
        remappings=[
            ('joint_states', 'joint_states'),
            ('imu/data', 'imu/data'),
            ('base_pose', 'base_pose'),
            ('joint_commands', 'joint_commands'),
            ('emergency_stop', 'emergency_stop'),
        ],
        output='screen',
    )
    
    # Create RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/humanoid_rs/config/humanoid.rviz'],
        condition=LaunchConfigurationEquals('use_rviz', 'true'),
    )
    
    # Log information
    log_info = LogInfo(
        msg=[
            'Starting humanoid robot node:\n',
            '  Node name: ', LaunchConfiguration('node_name'), '\n',
            '  Namespace: ', LaunchConfiguration('namespace'), '\n',
            '  Clock mode: ', LaunchConfiguration('clock_mode'),
        ]
    )
    
    return LaunchDescription([
        declare_node_name,
        declare_namespace,
        declare_clock_mode,
        declare_joint_state_rate,
        declare_imu_rate,
        declare_pose_rate,
        declare_real_time_factor,
        log_info,
        humanoid_node,
    ])
