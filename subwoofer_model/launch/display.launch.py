from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Should the joint state publisher be ran?
    gui_arg = DeclareLaunchArgument(name='gui',
                                    default_value='true',
                                    choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)

    # Which RViz2 config should be used.
    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=PathJoinSubstitution([FindPackageShare('subwoofer_model'), 'rviz', 'urdf.rviz']),
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # Default model location.
    ld.add_action(DeclareLaunchArgument(name='model', default_value=PathJoinSubstitution(['urdf', 'subwoofer.xacro']),
                                        description='Path to robot urdf file relative to package root'))

    # Use the urdf_launch package launch to start the display window.
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'subwoofer_model',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    return ld
