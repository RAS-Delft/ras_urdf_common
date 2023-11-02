from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    # Set up some parameters
    ras_urdf_common_path = FindPackageShare('ras_urdf_common')
    default_model_path = PathJoinSubstitution(['urdf', 'titoneri.urdf.xacro'])

    # Start joint_state_publisher
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='RAS_TN_DB'
    ))


    # Start joint_state_publisher
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='RAS_TN_PU'
    ))

    # Start RViz2
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([ras_urdf_common_path, 'rviz', 'urdf.rviz'])],
    ))

    ### Add the parts from the original description.launch.py
    robot_description_content1 = ParameterValue(Command(['xacro ',
                                                        PathJoinSubstitution([ras_urdf_common_path, default_model_path]),
                                                        ' ','vesselcolor:="0 0.51 1 1"',
                                                        ' ','vesselid:="RAS_TN_DB"']),
                                                        value_type=str)
    
    robot_description_content2 = ParameterValue(Command(['xacro ',
                                                        PathJoinSubstitution([ras_urdf_common_path, default_model_path]),
                                                        ' ','vesselcolor:="1 0.51 1 1"'
                                                        ' ','vesselid:="RAS_TN_PU"']),
                                                        value_type=str)
    
    robot_state_publisher_node1 = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    namespace='RAS_TN_DB',
                                    parameters=[{
                                          'robot_description': robot_description_content1,
                                    }])
    
    robot_state_publisher_node2 = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    namespace='RAS_TN_PU',
                                    parameters=[{
                                      'robot_description': robot_description_content2,
                                    }])
    
    robot_state_publisher_node3 = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    parameters=[{
                                      'robot_description': robot_description_content1,
                                        }])

    robot_state_publisher_node4 = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    parameters=[{
                                      'robot_description': robot_description_content2,
                                        }])

    ld.add_action(robot_state_publisher_node1)
    ld.add_action(robot_state_publisher_node2)

    return ld