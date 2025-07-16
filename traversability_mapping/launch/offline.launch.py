import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser


def generate_launch_description():

  # Configure environment
  stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
  stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

  # Simulated time
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  # Nodes Configurations
  config_file = os.path.join(get_package_share_directory('lego_loam'), 'config', 'loam_config.yaml')
  rviz_config = os.path.join(get_package_share_directory('traversability_mapping'), 'rviz', 'traversability_mapping.rviz')

  # Tf transformations

  transform_cam_init2map= Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_init_to_map',
    arguments=['0', '0', '0', '1.570795', '0', '1.570795', 'map', 'camera_init'],
    parameters=[{'use_sim_time': use_sim_time}]
  )

  transform_base_link2cam = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_camera',
    arguments=['0', '0', '0', '-1.570795', '-1.570795', '0', 'camera', 'base_link'],
    parameters=[{'use_sim_time': use_sim_time}]
  )


  transform_hesai = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='hesai_to_base_link',
    arguments=['0', '0', '0', '0', '0', '0','base_link','hesai_lidar'],
    parameters=[{'use_sim_time': use_sim_time}]
  )

  # LeGO-LOAM
  lego_loam_node = Node(
    package='lego_loam',
    executable='lego_loam',
    output='screen',
    parameters=[config_file, {'use_sim_time': use_sim_time}]
  )


  tfilter_node = Node(
      package='traversability_mapping',
      executable='traversability_filter',
      name='traversability_filter',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}]
  )

  tmapping_node = Node(
      package='traversability_mapping',
      executable='traversability_map',
      name='traversability_map',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time}]
  )
  
  # Rviz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config],
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}]
  )

  ld = LaunchDescription()
  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(stdout_colorized_envvar)
  ld.add_action(transform_cam_init2map)
  ld.add_action(transform_base_link2cam)

  ld.add_action(transform_hesai)

  # Add nodes
  ld.add_action(lego_loam_node)

  ld.add_action(rviz_node)
  ld.add_action(tfilter_node)
  ld.add_action(tmapping_node)

  return ld
