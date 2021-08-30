import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import xacro
import yaml

target_pkg = "tesseract_ros_examples"

def to_urdf(xacro_path, urdf_path=None, mappings=None):
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = tempfile.mktemp(prefix='%s_' % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=mappings)
    # open the output file
    print(urdf_path)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file

def load_file(package_name, file_path, mappings=None):
    package_path = get_package_share_directory(package_name)  # get package filepath
    absolute_file_path = os.path.join(package_path, file_path)
    temp_urdf_filepath = absolute_file_path.replace('.xacro', '')
    absolute_file_path = to_urdf(absolute_file_path, temp_urdf_filepath, mappings)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Robot description
    robot_description_config = load_file(target_pkg, 'urdf/irb2400_iiwa_cowork.xacro')
    robot_description = {'robot_description': robot_description_config}
    # Semantic description
    robot_description_semantic_config = load_file(target_pkg, 'config/scene_graph_example.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    scene_graph_example_node = Node(
            package='tesseract_ros_examples',
            executable=target_pkg+'_scene_graph_example_node',
            prefix='xterm -e gdb --args',
            output='screen',
            parameters=[robot_description, robot_description_semantic]
    )

    return LaunchDescription([
        scene_graph_example_node,
        ])
