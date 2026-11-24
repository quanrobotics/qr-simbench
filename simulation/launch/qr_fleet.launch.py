# QR-Sim fleet launch: one Gazebo server, N namespaced TurtleBot3 + Nav2 stacks.
# Derived from nav2_bringup cloned_multi_tb3_simulation_launch.py (Apache-2.0), adding a configurable
# robot SDF (ground-truth pose publisher) and headless server control.
#
# ros2 launch simulation/launch/qr_fleet.launch.py world:=... map:=... params_file:=... \
#     robot_sdf:=... robots:="amr_1={x: -5.0, y: 2.1, yaw: 0.0}; amr_2={x: -5.0, y: -0.4, yaw: 0.0}"

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    robot_sdf = LaunchConfiguration('robot_sdf')

    args = [
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('robot_sdf'),
        DeclareLaunchArgument('robots', default_value=''),
    ]
    world_sdf = tempfile.mktemp(prefix='qr_fleet_', suffix='.sdf')
    xacro = ExecuteProcess(cmd=['xacro', '-o', world_sdf, 'headless:=true', world])
    server = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world_sdf], output='screen')
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda _: os.path.exists(world_sdf) and os.remove(world_sdf))]))

    # Per robot: the nav2_bringup tb3 simulation stack, inlined so its gz bridge can drop /clock
    # (stock: one /clock bridge per robot; N publishers made sim time jump back and cleared TF).
    qr_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bridge_cfg = os.path.join(qr_dir, 'config', 'qr_robot_bridge.yaml')
    with open(os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')) as f:
        robot_description = f.read()
    remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    groups = [Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                   arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')]
    # Bring robot stacks up one after another (STAGGER_S apart): starting many Nav2 stacks at once
    # starves their lifecycle bonds on a desktop CPU and leaves some robots never ready.
    STAGGER_S = float(os.environ.get('QRSB_BRINGUP_STAGGER_S', '6.0'))
    bringup_launch = os.environ.get('QRSB_NAV_BRINGUP') or os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
    for k, (name, pose) in enumerate(ParseMultiRobotPose('robots').value().items()):
        groups.append(TimerAction(period=STAGGER_S * k, actions=[GroupAction([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', namespace=name, output='screen', remappings=remap,
                 parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={'namespace': name, 'use_namespace': 'True', 'slam': 'False',
                                  'map': map_yaml, 'use_sim_time': 'True', 'params_file': params_file,
                                  'autostart': 'True', 'use_composition': 'True',
                                  'use_respawn': 'False'}.items()),
            Node(package='ros_gz_bridge', executable='parameter_bridge', namespace=name, output='screen',
                 parameters=[{'config_file': bridge_cfg, 'expand_gz_topic_names': True,
                              'use_sim_time': True}]),
            Node(package='ros_gz_sim', executable='create', namespace=name, output='screen',
                 arguments=['-name', name, '-string',
                            Command([FindExecutable(name='xacro'), ' namespace:=', name, ' ', robot_sdf]),
                            '-x', str(pose['x']), '-y', str(pose['y']), '-z', str(pose.get('z', 0.01)),
                            '-R', '0.0', '-P', '0.0', '-Y', str(pose['yaw'])]),
        ])]))

    ld = LaunchDescription(args)
    ld.add_action(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')))
    ld.add_action(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(sim_dir)))
    ld.add_action(xacro)
    ld.add_action(server)
    ld.add_action(cleanup)
    for g in groups:
        ld.add_action(g)
    return ld
