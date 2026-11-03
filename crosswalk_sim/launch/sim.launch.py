"""Crosswalk simulation: Gazebo Harmonic world + delivery robot + ROS 2 bridges.

  ros2 launch crosswalk_sim sim.launch.py [scenario:=default|easy|hard|runner_test|<file.yaml>]
                                          [params:="cars.rate=12 red_runner.probability=0.5"]
                                          [seed:=42] [gui:=true] [rviz:=true]
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from crosswalk_sim.config import load_config
from crosswalk_sim.world_generator import generate_world


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    share = get_package_share_directory('crosswalk_sim')
    cfg = load_config(os.path.join(share, 'config', 'scenarios'),
                      LaunchConfiguration('scenario').perform(context),
                      LaunchConfiguration('params').perform(context),
                      LaunchConfiguration('seed').perform(context))
    world_name = cfg.setdefault('world_name', 'crosswalk')

    out_dir = os.path.join(tempfile.gettempdir(), 'crosswalk_sim')
    os.makedirs(out_dir, exist_ok=True)
    world_path = generate_world(cfg, os.path.join(out_dir, 'crosswalk_world.sdf'))
    with open(os.path.join(out_dir, 'scenario_effective.yaml'), 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    # ---------------------------------------------------------------- robot
    r = cfg['robot']
    cam, lid = r['camera'], r['lidar']
    xacro_file = os.path.join(get_package_share_directory('delivery_robot_description'),
                              'urdf', 'delivery_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings={
        'camera_width': str(cam['width']), 'camera_height': str(cam['height']),
        'camera_fps': str(cam['fps']), 'camera_hfov': str(cam['hfov']),
        'camera_pitch': str(cam.get('pitch', 0.0)),
        'lidar_samples': str(lid['samples']), 'lidar_range': str(lid['range']),
        'lidar_rate': str(lid['rate']), 'lidar_vertical_samples': str(lid['vertical_samples']),
        'lidar_vertical_fov': str(lid.get('vertical_fov', 0.5236)),
        'max_linear_velocity': str(r.get('max_linear_velocity', 1.6)),
        'max_angular_velocity': str(r.get('max_angular_velocity', 2.0)),
    }).toxml()
    sx, sy, syaw = (float(v) for v in r['spawn'])

    # --------------------------------------------------------------- gazebo
    gui = _truthy(context, 'gui')
    gz_cmd = ['gz', 'sim', '-r', '-v', LaunchConfiguration('gz_verbosity').perform(context)]
    if not gui:
        gz_cmd += ['-s', '--headless-rendering']
    gz_cmd.append(world_path)
    plugin_path = os.path.join(get_package_prefix('crosswalk_sim'), 'lib')
    old = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    gazebo = ExecuteProcess(
        cmd=gz_cmd, output='screen',
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': plugin_path + (os.pathsep + old if old else '')})

    sim_time = {'use_sim_time': True}
    actions = [
        LogInfo(msg=f"[crosswalk_sim] world: {world_path}  |  effective scenario: "
                    f"{os.path.join(out_dir, 'scenario_effective.yaml')}"),
        LogInfo(msg=f"[crosswalk_sim] seed={cfg.get('seed')}  pedestrians: crossing "
                    f"{cfg['pedestrians']['crossing_rate']}/min, sidewalk {cfg['pedestrians']['sidewalk_rate']}/min"
                    f"  |  cars: {cfg['cars']['rate']}/min/lane  |  red runner: p="
                    f"{cfg['red_runner']['probability']} window={cfg['red_runner']['window']} s"),
        gazebo,
        RegisterEventHandler(OnProcessExit(target_action=gazebo,
                                           on_exit=[EmitEvent(event=Shutdown(reason='gazebo exited'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}, sim_time]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', world_name, '-name', r['name'], '-topic', 'robot_description',
                        '-x', str(sx), '-y', str(sy), '-z', '0.02', '-Y', str(syaw)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='crosswalk_bridge',
             output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}, sim_time]),
        Node(package='ros_gz_image', executable='image_bridge', name='camera_bridge', output='screen',
             arguments=['/camera/image_raw'], parameters=[sim_time]),
        Node(package='crosswalk_sim', executable='crosswalk_monitor', output='screen',
             parameters=[sim_time, {
                 'lane_width': float(cfg['world']['lane_width']),
                 'sidewalk_width': float(cfg['world']['sidewalk_width']),
                 'crosswalk_width': float(cfg['world']['crosswalk_width']),
                 'stop_line_offset': float(cfg['world']['stop_line_offset']),
                 'road_length': float(cfg['world']['road_length']),
                 'robot_spawn': [sx, sy, syaw],
                 'goal': [float(v) for v in r['goal']],
             }]),
    ]
    if cfg['world'].get('overview_camera', False):
        actions.append(Node(package='ros_gz_image', executable='image_bridge', name='overview_bridge',
                            output='screen', arguments=['/overview_camera/image'], parameters=[sim_time]))
    if _truthy(context, 'rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', output='log',
                            arguments=['-d', os.path.join(share, 'rviz', 'crosswalk.rviz')],
                            parameters=[sim_time]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='default',
                              description='Scenario name from config/scenarios or a path to a YAML file'),
        DeclareLaunchArgument('params', default_value='',
                              description='Overrides, e.g. "cars.rate=12 red_runner.window=[2,6]"'),
        DeclareLaunchArgument('seed', default_value='',
                              description='Random seed (overrides the scenario file; 0 = random)'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('gz_verbosity', default_value='2', description='Gazebo verbosity 0-4'),
        OpaqueFunction(function=_launch_setup),
    ])
