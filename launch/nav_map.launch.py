from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Map file argument
    declare_map = DeclareLaunchArgument(
        "map",
        default_value="/home/aech/ros2_ws/src/ros2-slam-auto-navigation/maps/mymap2.yaml",
        description="Full path to map yaml file"
    )

    map_file = LaunchConfiguration("map")

    # Launch Nav2 bringup
    nav2_bringup = Node(
        package="nav2_bringup",
        executable="navigation_launch.py",   # valid for Humble
        output="screen",
        parameters=[{
            "use_sim_time": False
        }],
        arguments=[
            f"map:={map_file}"
        ]
    )

    # Your goal sender node
    goal_sender = Node(
        package="your_package",
        executable="goal",   # corresponds to your goal.py
        output="screen"
    )

    return LaunchDescription([
        declare_map,
        nav2_bringup,
        goal_sender
    ])
