import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('s500_autonomous_slam')
    pkg_mrtsp_exploration = get_package_share_directory('mrtsp_exploration_ros2')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz = LaunchConfiguration('rviz', default='false')
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

    # ==================== Nav2 Navigation ====================
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_real.yaml')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'use_collision_monitor': 'True'
        }.items()
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

    # ==================== MRTSP Exploration (delayed 15s) ====================
    mrtsp_params_file = os.path.join(pkg_dir, 'config', 'mrtsp_real.yaml')
    mrtsp_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_mrtsp_exploration, 'launch', 'explore.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': mrtsp_params_file
                }.items()
            )
        ]
    )

    # ==================== RViz (optional, off by default for headless RPi5) ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz (enable on desktop, disable on headless RPi5)'),
        DeclareLaunchArgument('takeoff_altitude', default_value='2.0', description='Takeoff altitude in meters'),
        DeclareLaunchArgument('max_velocity', default_value='0.5', description='Maximum velocity in m/s'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0', description='Serial port for Pixhawk UART'),
        DeclareLaunchArgument('baud_rate', default_value='921600', description='Baud rate for micro-XRCE-DDS'),
        micro_xrce_agent,
        robot_state_publisher,
        odom_tf_publisher_px4,
        camera_node,
        rtabmap_slam,
        nav2_launch,
        offboard_control_node,
        mrtsp_launch,
        rviz_node
    ])
