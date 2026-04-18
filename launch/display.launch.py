from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile
import xacro


def generate_launch_description():
    declare_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Зайпуск Rviz'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time')

    pkg_path = get_package_share_directory('robot_pkg')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    bridge_config = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    urdf_content = xacro.process_file(xacro_file).toxml()
    rviz_config_file = os.path.join(pkg_path, 'config', 'robot.rviz')
    world_file = os.path.join(pkg_path, 'worlds', 'Depot.sdf')

    
    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    tmp.write(urdf_content)
    tmp.close()
    urdf_path = tmp.name

    gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge_config,
                'qos_overrides./tf_static.publisher.durability':'transient_local',
            }],
            output='screen')

    spawn_model_node = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'diff_drive',
                '-topic', "robot_description",
                '-x', '0',
                '-y', '0',
                '-z', '0.2'])

    gz_sim_exec = ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', f'gz_args:={world_file}'],
            output='screen')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='both',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}])

    joint_state_publisher_gui_node =  Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='both',
            parameters=[{'robot_description': urdf_content}])

    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='both',
            parameters=[{'robot_description': urdf_content}])

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': urdf_content},
            {'use_sim_time': use_sim_time}]
        )
    
    map_to_odom = Node(
            package='robot_pkg',
            executable='tf_to_rviz',
            name='map_to_base_link',
            output='both'
        )
    
    fixes_lidar = Node(
            package='robot_pkg',
            executable='fixed_lidar',
            name='fixed_lidar_frame_id',
            output='both'
        )


    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_arg,
        robot_state_publisher_node,
        fixes_lidar,
        rviz_node,
        gz_sim_exec,
        gz_bridge_node,
        spawn_model_node
        
    ])  