import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージディレクトリを取得
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')

    # Xacroファイルのパス
    xacro_file = os.path.join(pkg_share, 'src/description/pioneer_robot.xacro')

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="p1", description="robots id"),
        DeclareLaunchArgument("x", default_value="0.0", description="x pos"),
        DeclareLaunchArgument("y", default_value="0.0", description="y pos"),
        DeclareLaunchArgument("t", default_value="0.0", description="theta"),
        
        # Gazeboやシミュレーション用の時間を有効化
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        # robot_state_publisherを起動（Xacroを動的に処理）
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command([
                    "xacro ", 
                    xacro_file, 
                    " robot_id:=", 
                    LaunchConfiguration("robot_id"),
                    " x:=", 
                    LaunchConfiguration("x"),
                    " y:=", 
                    LaunchConfiguration("y"),
                    " t:=", 
                    LaunchConfiguration("t")
                ]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }]
        ),
    ])
