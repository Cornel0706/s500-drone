import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('s500_autonomous_slam')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude', default='2.0')
    max_velocity = LaunchConfiguration('max_velocity', default='0.5')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyAMA0')
    baud_rate = LaunchConfiguration('baud_rate', default='921600')

    urdf_file = os.path.join(pkg_dir, 'urdf', 's500_drone.urdf.xacro')

    # ==================== PX4 Communication ====================
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', serial_port, '-b', baud_rate],
        output='screen'
    )

    # ==================== Robot Description ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )

    # ==================== Odometry TF ====================
    odom_tf_publisher_px4 = Node(
        package='s500_autonomous_slam',
        executable='odom_tf_publisher_px4',
        name='odom_tf_publisher_px4',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ==================== Camera Driver ====================
    # --- Luxonis OAK-D S2 ---
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py'))
    )

    # ==================== RTAB-Map Visual SLAM ====================
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'approx_sync': True,
            'queue_size': 30,
            'Grid/FromDepth': 'true',
            'Reg/Force3DoF': 'true',
            'Optimizer/Slam2D': 'true',
            'Grid/RayTracing': 'true',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MinGroundHeight': '0.04',
            'Grid/CellSize': '0.05',
            'Rtabmap/DetectionRate': '2.0',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            # Luxonis OAK-D S2 topics:
            ('rgb/image', '/camera/rgb/image_raw'),
            ('rgb/camera_info', '/camera/rgb/camera_info'),
            ('depth/image', '/camera/stereo/image_raw')
        ]
    )

    # ==================== Offboard Control ====================
    offboard_control_node = Node(
        package='s500_autonomous_slam',
        executable='offboard_control_node',
        name='offboard_control_node',
        output='screen',
        parameters=[{
            'takeoff_altitude': takeoff_altitude,
            'max_velocity': max_velocity,
            'auto_takeoff': True,
            'use_sim_time': use_sim_time
        }]
    )

    # ==================== Teleop (manual control) ====================
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --'
    )

    # ==================== RViz ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        DeclareLaunchArgument('takeoff_altitude', default_value='2.0', description='Takeoff altitude in meters'),
        DeclareLaunchArgument('max_velocity', default_value='0.5', description='Maximum velocity in m/s'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0', description='Serial port for Pixhawk UART'),
        DeclareLaunchArgument('baud_rate', default_value='921600', description='Baud rate for micro-XRCE-DDS'),
        micro_xrce_agent,
        robot_state_publisher,
        odom_tf_publisher_px4,
        camera_node,
        rtabmap_slam,
        offboard_control_node,
        teleop_node,
        rviz_node
    ])
