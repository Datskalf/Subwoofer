# ROS2 Launch file that converts xacro -> urdf, and spawns urdf into sdf world

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro
import os.path

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    arg1 = 1
    arg2 = 2
    
    description_path = os.path.join(get_package_share_directory("subwoofer_model"))

    xacro_file = os.path.join(description_path, "urdf", "subwoofer.urdf.xacro")

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc,  mappings={"arg1": arg1, "arg2": arg2})

    ignition_spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=["-string", doc.toxml(), "-name", "subwoofer_model", "-allow_renaming", "true"],
    )


    return LaunchDescription(
        [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("ros_ign_gazebo"),
                            "launch",
                            "ign_gazebo.launch.py",
                        )
                    ]
                ),
                launch_arguments=[("ign_args", [" -v 5 subwoofer_empty.sdf"])],
            ),
            ignition_spawn_entity,
            
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )
