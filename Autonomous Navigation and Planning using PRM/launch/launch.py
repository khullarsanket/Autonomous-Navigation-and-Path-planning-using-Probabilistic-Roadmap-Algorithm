from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from project4d.disc_robot import load_disc_robot




def generate_launch_description():
    # robot_name_arg = DeclareLaunchArgument('robot_name') ## argument to enter at the terminal for the robot name
    

    robot_file = 'ideal.robot'
    world_file = 'pillars.world'
    # robot_file_arg = DeclareLaunchArgument('robot_name')
    # robot_file = LaunchConfiguration('robot_name')

    robot_name_file_path= f'/home/lab2004/project_ws/src/project4a/project4a/{robot_file}'
    world_name_file_path= f'/home/lab2004/project_ws/src/project4d/project4d/{world_file}'

    # robot_name_file_path = Command(["echo /home/lab2004/project_ws/src/project4a/project4a/", robot_file])

    robot = load_disc_robot(robot_name_file_path)
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot['urdf']}],
        )
    velocity_translator_node = Node(
            package='project4d',
            executable='velocity_translator',
            name='velocity_translator',
            output='screen',
            parameters=[{'robot_name':robot_name_file_path}]
        )
    
    differential_drive_simulator_node = Node(
            package='project4d',
            executable='differential_drive_simulator',
            name='differential_drive_simulator',
            output='screen',
            parameters=[{'robot_name':robot_name_file_path},{'world_name':world_name_file_path}],
        )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        )
    return LaunchDescription([
        robot_state_publisher_node,
        velocity_translator_node,
        differential_drive_simulator_node,
        rviz_node,
        

    ])