from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value='/home/taher/ros2_ws/src/ros2-slam-auto-navigation/config/mapper_params_online_async.yaml',
        description='Path to the SLAM toolbox parameter file.'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time.'
    )

    # Include SLAM Toolbox launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # Include Nav2 Bringup launch
    # nav2_bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('nav2_bringup'),
    #             'launch',
    #             'navigation_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'map': PathJoinSubstitution([
    #         FindPackageShare('ros2_slam_auto_navigation'),
    #         'maps',
    #         'mymap.yaml'
    #     ])
    #     }.items()
    # )

    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
    )

    ekf_node = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter",
    output="screen",
    parameters=[os.path.join(
        get_package_share_directory('ros2_slam_auto_navigation'),
        'config',
        'ekf.yaml'
    )]
)

    return LaunchDescription([
        ekf_node,
        slam_params_file_arg,
        use_sim_time_arg,
        slam_toolbox_launch,
        # nav2_bringup_launch,
        rviz_node
    ])
