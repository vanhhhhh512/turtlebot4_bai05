"""Saved maze map, AMCL/Nav2, RGB YOLO and a sequential three-goal program."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('bai05_turtlebot4')
    bai04 = get_package_share_directory('bai04_turtlebot4')
    sim = get_package_share_directory('turtlebot4_ignition_bringup')
    nav = get_package_share_directory('turtlebot4_navigation')

    def include(root, name, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(root, 'launch', name)), launch_arguments=args.items())

    output = LaunchConfiguration('output_dir')

    def simulator(context):
        generated = Path(output.perform(context)) / 'world'
        generated.mkdir(parents=True, exist_ok=True)
        world = generated / 'maze_yolo.sdf'
        world.write_text(Path(share, 'worlds', 'maze_yolo.sdf').read_text().replace(
            '__BAI05_MODELS__', os.path.join(share, 'models')))
        return [ExecuteProcess(cmd=['ign', 'gazebo', '-s', str(world), '-r', '-v', '2'],
                               additional_env={'DISPLAY': LaunchConfiguration('server_display').perform(context)},
                               output='screen'),
                TimerAction(period=35.0, actions=[ExecuteProcess(
                    cmd=['ign', 'gazebo', '-g', '-v', '2', '--gui-config',
                         os.path.join(share, 'config', 'gazebo_gui.config')], output='screen',
                    condition=IfCondition(LaunchConfiguration('show_gazebo_gui')))])]

    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser('~/bai05_run')),
        DeclareLaunchArgument('run_route', default_value='true'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('show_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('yolo', default_value='true'),
        DeclareLaunchArgument('show_yolo_window', default_value='true'),
        DeclareLaunchArgument('yolo_model', default_value=os.path.join(
            share, 'models', 'yolo_world_furniture.torchscript')),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('server_display', default_value=os.environ.get('DISPLAY', ':0')),
        DeclareLaunchArgument('map', default_value=os.path.join(bai04, 'maps', 'map.yaml')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join([
            os.path.join(share, 'worlds'), os.path.join(share, 'models'),
            os.path.join(sim, 'worlds'), '/opt/ros/humble/share',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', ':'.join([
            os.path.join(get_package_share_directory('turtlebot4_ignition_gui_plugins'), 'lib'),
            os.path.join(get_package_share_directory('irobot_create_ignition_plugins'), 'lib')])),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ':'.join([
            '/opt/ros/humble/lib', os.environ.get('LD_LIBRARY_PATH', ''),
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')])),
        OpaqueFunction(function=simulator),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='bai05_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        include(sim, 'turtlebot4_spawn.launch.py', {
            'world': 'maze', 'model': 'standard',
            'x': '0.0', 'y': '0.0', 'yaw': '0.0', 'rviz': 'false'}),
        TimerAction(period=10.0, actions=[include(nav, 'localization.launch.py', {
            'map': LaunchConfiguration('map'), 'use_sim_time': 'true',
            'params': os.path.join(share, 'config', 'localization.yaml')}),
        ]),
        TimerAction(period=20.0, actions=[include(nav, 'nav2.launch.py', {
            'use_sim_time': 'true', 'params_file': os.path.join(share, 'config', 'nav2.yaml')})]),
        TimerAction(period=42.0, actions=[Node(package='rviz2', executable='rviz2', arguments=[
            '-d', os.path.join(share, 'rviz', 'navigation.rviz')],
            parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('show_rviz'))),
        Node(package='bai05_turtlebot4', executable='yolo_detector.py', output='screen',
             parameters=[{'use_sim_time': True, 'output_dir': output,
                          'model': ParameterValue(LaunchConfiguration('yolo_model'), value_type=str),
                          'confidence': 0.12, 'observation_confidence': 0.012,
                          'imgsz': 960,
                          'show_window': ParameterValue(LaunchConfiguration('show_yolo_window'), value_type=bool),
                          'device': ParameterValue(LaunchConfiguration('device'), value_type=str)}],
             condition=IfCondition(LaunchConfiguration('yolo')))]),
        TimerAction(period=55.0, actions=[Node(package='bai05_turtlebot4', executable='navigate_goals.py', output='screen',
             parameters=[{'use_sim_time': True, 'output_dir': output,
                          'goals_file': os.path.join(share, 'config', 'goals.yaml')}],
             condition=IfCondition(LaunchConfiguration('run_route')))]),
    ])
