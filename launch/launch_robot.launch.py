import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    # # Declare position arguments
    # declare_x_cmd = DeclareLaunchArgument("x", default_value="0.0")
    # declare_y_cmd = DeclareLaunchArgument("y", default_value="0.0")
    # declare_z_cmd = DeclareLaunchArgument("z", default_value="0.1")
    # declare_Y_cmd = DeclareLaunchArgument("Y", default_value="-1.578")   # yaw

    # # LaunchConfigurations for each
    # x = LaunchConfiguration("x")
    # y = LaunchConfiguration("y")
    # z = LaunchConfiguration("z")
    # Y = LaunchConfiguration("Y")

    # Define the package name for easier reference
    package_name = 'ros2_slam_auto_navigation'

    # Declare the 'world_file' launch argument
    # This allows the user to specify the path to the Gazebo world file to be loaded
    # declare_world_file_cmd = DeclareLaunchArgument(
    #     'world_file',
    #     default_value=os.path.join(get_package_share_directory(package_name), 'worlds', 'arena.world'),
    #     description='Full path to the world file to load'
    # )

    # Access the 'world_file' argument during runtime using LaunchConfiguration
    # world_file = LaunchConfiguration('world_file')

    # Include the Robot State Publisher (RSP) launch file
    # This sets up the robot's description (URDF) and enables simulation time and ROS 2 control
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Specify the Twist Mux configuration file path
    # Twist Mux is used to manage velocity command inputs from different sources
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    # Launch the Twist Mux node with the specified configuration file
    # Remap the output command velocity topic to match the robot's controller
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Specify the Gazebo parameters configuration file path

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controller.yaml')


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        arguments=[{'robot_description:' : robot_description},
                   controller_params_file]
    )

    delayed_controller_manager = TimerAction(period= 3.0, actions=[controller_manager])
    

    # Launch the diff_drive_controller for controlling the robot's differential drive system
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action= controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    # Launch the joint_state_broadcaster for publishing the robot's joint states
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action= controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    # Combine all the nodes and arguments into a LaunchDescription
    # This ensures all components are launched together
    return LaunchDescription([
        # declare_world_file_cmd,  # Declare the world file argument for customization
        # declare_x_cmd,
        # declare_y_cmd,
        # declare_z_cmd,
        # declare_Y_cmd,
        rsp,  # Robot State Publisher launch
        twist_mux,  # Twist Mux node for velocity command management
        # gazebo,  # Gazebo simulator with world and parameters
        # spawn_entity,  # Spawn the robot in the Gazebo world
        delayed_controller_manager,
        delayed_diff_drive_spawner,  # Start the differential drive controller
        delayed_joint_broad_spawner # Start the joint state broadcaster
        
    ])
