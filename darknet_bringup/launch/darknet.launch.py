# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():

    darknet_bringup_path = get_package_share_directory("darknet_bringup")

    #
    # ARGS
    #

    network_config = LaunchConfiguration("network_config")
    network_config_cmd = DeclareLaunchArgument(
        "network_config",
        default_value=os.path.join(
            darknet_bringup_path, "config", "yolov3-tiny.cfg"),
        description="Network configuration file (.cfg)")

    weights = LaunchConfiguration("weights")
    weights_cmd = DeclareLaunchArgument(
        "weights",
        default_value=os.path.join(
            darknet_bringup_path, "config", "yolov3-tiny.weights"),
        description="Weights file (.weights)")

    class_names = LaunchConfiguration("class_names")
    class_names_cmd = DeclareLaunchArgument(
        "class_names",
        default_value=os.path.join(
            darknet_bringup_path, "config", "coco.names"),
        description="Class names file (.names)")

    enalbe_darknet = LaunchConfiguration("enalbe_darknet")
    enalbe_darknet_cmd = DeclareLaunchArgument(
        "enalbe_darknet",
        default_value="True",
        description="Wheter to start darknet enabled")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.30",
        description="Minimum probability of a detection to be published")

    nms_threshold = LaunchConfiguration("nms_threshold")
    nms_threshold_cmd = DeclareLaunchArgument(
        "nms_threshold",
        default_value="0.45",
        description="Non-maximal Suppression threshold - controls filtering of overlapping boxes")

    show_debug_image = LaunchConfiguration("show_debug_image")
    show_debug_image_cmd = DeclareLaunchArgument(
        "show_debug_image",
        default_value="True",
        description="Wheter show image with bounding boxes")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/image_raw",
        description="Name of the input image topic")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="darknet",
        description="Namespace")

    #
    # NODES
    #

    detector_node_cmd = Node(
        package="darknet_ros",
        executable="detector_node",
        name="detector_node",
        namespace=namespace,
        parameters=[{"network.config": network_config,
                     "network.weights": weights,
                     "network.class_names": class_names,
                     "detection.enable": enalbe_darknet,
                     "detection.threshold": threshold,
                     "detection.nms_threshold": nms_threshold}],
        remappings=[("images", input_image_topic)]
    )

    detection_visualizer_cmd = Node(
        package="detection_visualizer",
        executable="detection_visualizer",
        name="detection_visualizer",
        namespace=namespace,
        parameters=[{"class_names": class_names}],
        remappings=[("images", input_image_topic)],
        condition=IfCondition(PythonExpression([show_debug_image]))
    )

    image_view_cmd = Node(
        package="image_view",
        executable="image_view",
        name="image_view",
        namespace=namespace,
        remappings=[("image", "dbg_images")],
        condition=IfCondition(PythonExpression([show_debug_image]))
    )

    ld = LaunchDescription()

    ld.add_action(network_config_cmd)
    ld.add_action(weights_cmd)
    ld.add_action(class_names_cmd)
    ld.add_action(enalbe_darknet_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(nms_threshold_cmd)
    ld.add_action(show_debug_image_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detector_node_cmd)
    ld.add_action(detection_visualizer_cmd)
    ld.add_action(image_view_cmd)

    return ld
