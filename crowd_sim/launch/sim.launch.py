import os
import sys
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('crowd_sim')
    params_file = os.path.join(pkg_share, 'config', 'sim_params.yaml')
    world_file = os.path.join(pkg_share, 'config', 'world.yaml')
    template_path = os.path.join(pkg_share, 'gazebo_sim', 'sim_world.sdf.template')
    # the generated world goes to a writable temp dir, not the install tree
    world_path = os.path.join(tempfile.gettempdir(), 'crowd_sim_world.sdf')

    # build the gazebo world from the template + geometry in world.yaml
    sys.path.insert(0, os.path.join(pkg_share, 'gazebo_sim'))
    from build_sdf import build_sdf
    build_sdf(world_file, template_path, world_path)

    # Start Gazebo using ros_gz_sim's own launch file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),  # -r = start unpaused
    )

    # run ros-gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
          '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
          '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
          '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
          '/world/crowd_world/create@ros_gz_interfaces/srv/SpawnEntity',
          '/world/crowd_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
          '/world/crowd_world/remove@ros_gz_interfaces/srv/DeleteEntity',
        ],
        output='screen',
    )

    # start ros2 nodes
    controller_node = Node(
        package='crowd_sim',
        executable='controller',
        parameters=[{'use_sim_time': True}, {'print_scans': True}],
        output='screen',
    )

    crowd_node = Node(
        package='crowd_sim',
        executable='crowd',
        parameters=[params_file],
        output='screen'
    )

    # get rviz config
    rviz_config = os.path.join(pkg_share, 'rviz', 'crowd_config.rviz')
    # start rviz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([gz_sim, bridge, controller_node, crowd_node, rviz2])
