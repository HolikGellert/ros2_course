import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Path to the custom world file
    pkg_share = get_package_share_directory('ros2_course')
    world_file = os.path.join(pkg_share, 'maps', 'FINAL.world')

    # 2. Path to the TurtleBot3 SDF model (for spawning)
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    model_folder = 'turtlebot3_burger'
    urdf_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')

    # 3. Launch Gazebo with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # 4. Spawn the robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen',
    )

    # 5. Robot State Publisher (Required for RViz and TF)
    # Load the URDF file
    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf_path_publisher = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    with open(urdf_path_publisher, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
