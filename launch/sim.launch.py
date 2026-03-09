#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

#=====================================================================================================
#======================================= generate launch description ======================================
def generate_launch_description():

    px4_dir = os.path.expanduser('~/drone/PX4-Autopilot')
    
    # Get package directories
    pkg_visual_follow = get_package_share_directory('visual_follow_me')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Locate the world and model files
    world_file = os.path.join(pkg_visual_follow, 'config', 'worlds', 'visual_follow_me.sdf')
    model_path = os.path.join(pkg_visual_follow, 'config', 'models')
    
    config_file_path = os.path.join(pkg_visual_follow, 'launch', 'server.config')

    # --- Launch Arguments ---
    verbose_arg = DeclareLaunchArgument('verbose', default_value='True')
    headless_arg = DeclareLaunchArgument('headless', default_value='False')

    verbose = LaunchConfiguration('verbose')
    headless = LaunchConfiguration('headless')

    # --- Environment Variables ---
    # Set Gazebo resource path so it finds your custom models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
    	value=model_path
    )

    gz_server_config = SetEnvironmentVariable(
        name='GZ_SIM_SERVER_CONFIG_PATH',
        value=config_file_path
	)
    
#=========================================Gazebo Sim ==============================================================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': PythonExpression([
                f"'{world_file} -r'",
                " + (' -v' if '", verbose, "' == 'True' else '')",
                " + (' -s' if '", headless, "' == 'True' else '')"
            ])
        }.items()
    )    
    
#=====================================================================================================
#======================================= start px4 ==============================================================
    start_px4 = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'bash', '-c', 
            f'cd {px4_dir} && make px4_sitl gz_x500_gimbal'
        ],
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'visual_follow_me'
        },
        output='screen'
    )
#=====================================================================================================
#======================================= ros_gz_bridge ========================================================
    ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_camera',
            arguments=[
                # ROS -> GZ (Control)
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/cmd_path@geometry_msgs/msg/PoseArray]gz.msgs.Pose_V',
                
                # GZ -> ROS (Sensors)
                '/world/visual_follow_me/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/world/visual_follow_me/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
            ],
            remappings=[
                ('/world/visual_follow_me/model/x500_gimbal_0/link/camera_link/sensor/camera/image', '/camera/image_raw'),
                ('/world/visual_follow_me/model/x500_gimbal_0/link/camera_link/sensor/camera/camera_info', '/camera/camera_info')
            ],
            output='screen'
    )

#====================================================================================================
#========================================= dds agent ===================================================
    dds_agent_start = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 
            'bash', '-c', 
            f'MicroXRCEAgent udp4 -p 8888; exec bash'
        ],
        output='screen'
    )
#=====================================================================================================

#=====================================================================================================
#======================================= return launch description ======================================
    return LaunchDescription([
        verbose_arg,
        headless_arg,
        gz_resource_path,
        gz_server_config,
        gz_sim,
        start_px4,
        dds_agent_start,
        ros_gz_bridge,
    ])