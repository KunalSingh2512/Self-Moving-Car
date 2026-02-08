import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_car_description'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Process the URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_car.urdf.xacro')
    
    # Check if file exists to avoid confused errors
    if not os.path.exists(xacro_file):
        print(f"ERROR: URDF not found at {xacro_file}. Did you add the install rule?")
    
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # 2. Configure Gazebo (With ability to load a World later)
    # For now, we load the default empty world, but we add the verbose flag for debugging
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 3. Spawn the Robot
    # We add a slight Z-offset (0.15) to ensure wheels don't clip into the ground on spawn
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_car',
            '-z', '0.15' 
        ],
        output='screen'
    )

    # 4. Robot State Publisher (The "Brain" broadcaster)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. Joint State Publisher (Optional but recommended for Rviz visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher, # Added for completeness
        spawn_entity
    ])