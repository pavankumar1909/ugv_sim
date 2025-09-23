import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'ugv_sim'  # your package with URDF/Xacro

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    

    #  # World argument
    # world = LaunchConfiguration('world')
    # declare_world = DeclareLaunchArgument(
    #     'world',
    #     default_value=os.path.join(
    #         get_package_share_directory(package_name),
    #         'worlds',
    #         'empty.sdf'
    #     ),
    #     description='Gazebo world file'
    # )
    
    # Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', './src/ugv_sim/worlds/obstacles_1.sdf'],  # <-- change to your world file
        output='screen'
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot'],
        output='screen'
    )
    # ROS-Gazebo Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
        ],
    )
    # RViz2
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'robot_model_rviz.rviz'  # <-- create this config file inside your package
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        bridge,
        rsp,
        gazebo,
        spawn_entity,
        rviz
    ])
