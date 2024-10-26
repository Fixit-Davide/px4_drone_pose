import os
import yaml
import pathlib
import launch.actions
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

package_name = "drone_pose"

'''
Used to load parameters for composable nodes
'''
def dump_params(path, name):
    # Load the parameters specific to your ComposableNode
    with open(path, 'r') as file:
        return [yaml.safe_load(file)[name]['ros__parameters']]

def generate_launch_description():

  config = os.path.join(
          get_package_share_directory('vdb_mapping_ros2'),
          'config',
          'vdb_params.yaml'
          ),
  bridge_config = os.path.join(
          get_package_share_directory('drone_pose'),
          'params',
          'ros_bridge_config.yaml'
          ),
  return LaunchDescription([
    # define static tf for base_link and camera.
    # Node(
    #     package = "tf2_ros",
    #     executable = "static_transform_publisher",
    #     arguments = ["0.12", "0.03", "0.242", "0", "0", "0", "x500_depth_0/base_link", "x500_depth_0/OakD-Lite/base_link/StereoOV7251"],
    #     output='screen',
    #     parameters=[{
    #       'use_sim_time': True}],   
    # ),
    # define static tf for base_link and lidar.
    Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0.0", "0.0", "0.35", "0", "0", "0", "x500_lidar_0/base_link", "x500_lidar_0/link/lidar_2d_v2"],
        output='screen',
        parameters=[{
          'use_sim_time': True}], 
    ),
    # Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     # arguments=['/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
    #     #            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
    #     arguments=['/world/walls/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
    #               '/world/walls/model/x500_lidar_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    #               '/world/walls/model/x500_lidar_0/link/link/sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
    #     output='screen',
    #     remappings=[('/world/walls/clock', '/clock')],
    #     parameters=[{
    #       'use_sim_time': True}], 
    # ),

    ComposableNodeContainer(
        name='Container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vdb_mapping_ros2',
                plugin='vdb_mapping_ros2::vdb_mapping_ros2_component',
                name='vdb_mapping',
                parameters=[config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='drone_pose',
                plugin='px4_autonav::DronePose',
                name='drone_pose',
                parameters=[os.path.join(get_package_share_directory("drone_pose"), 'params', 'drone_pose.yaml')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='ros_gz_bridge',
                plugin='ros_gz_bridge::RosGzBridge',
                name='ros_gz_bridge',
                # arguments=['/world/walls/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                #   '/world/walls/model/x500_lidar_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                #   '/world/walls/model/x500_lidar_0/link/link/sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
                parameters=[{'config_file' : '/home/davide/ros2_drone_ws/src/PX4-ROS2-Drone-Composable/params/ros_bridge_config.yaml'}],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    ),
# , {'use_sim_time': True}
    Node(
       package='rviz2',
       executable='rviz2',
        parameters=[{
          'use_sim_time': True}],
    )
])