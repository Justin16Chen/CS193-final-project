import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cs193project')
    world = os.path.join(pkg_share, 'gazebo_sim', 'sensors.sdf')

    # Start Gazebo using ros_gz_sim's own launch file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),  # -r = start unpaused
    )

    # run ros-gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen',
    )

    # start ros2 nodes
    clock_node = Node(
        package='cs193project',
        executable='clock_reciever',
        parameters=[{'use_sim_time': True}, {'print_clock': False}],
        output='screen',
    )

    controller_node = Node(
        package='cs193project',
        executable='controller',
        parameters=[{'use_sim_time': True}, {'print_scans': True}],
        output='screen',
    )

    crowd_node = Node(
        package='cs193project',
        executable='crowd',
        parameters=[
          {'update_frequency': 20.},
          # spawn data
          {'num_agents': 25,},
          {'spawn_x_min': -10.},
          {'spawn_x_max': 10.},
          {'spawn_y_min': -10.},
          {'spawn_y_max': 10.},
          # agent data
          {'model.avoid_radius': 1.},
          {'model.anisotropy': 0.5},
          {'model.repel_strength': 0.5},
          {'model.repel_strength_falloff': 0.3},
          {'model.preferred_walk_speed': 1.3},
          {'model.max_walk_speed': 1.6},
          {'model.relaxation_time': 0.7},
        ],
        output='screen'
    )

    return LaunchDescription([gz_sim, bridge, clock_node, controller_node, crowd_node])
