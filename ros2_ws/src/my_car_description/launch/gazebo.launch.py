import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_car_description'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Process the URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_car.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Launch Gazebo (The Physics Engine)
    # We use the standard empty world for now
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 3. Spawn the Robot (The "Birth" of the car)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_car',
                   '-z', '0.1'], # Spawn slightly above ground to prevent clipping
        output='screen'
    )

    # 4. Robot State Publisher (The "Brain" broadcaster)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}] # IMPORTANT: Tells ROS to use Gazebo time, not Laptop time
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])