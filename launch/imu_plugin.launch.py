# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node


def get_plugin_name(context):
    return LaunchConfiguration("plugin_name").perform(context)


def get_plugin_configuration(context):
    with open(LaunchConfiguration("plugin_configuration_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def launch_setup(context, *args, **kwargs):
    plugin_name = get_plugin_name(context)
    plugin_configuration = get_plugin_configuration(context)
    component_container = get_component_container(context)

    if not component_container:
        node = Node(
            package="romea_localisation_imu_plugin",
            executable="imu_localisation_plugin_node",
            name=plugin_name,
            parameters=[plugin_configuration],
        )
        return [node]
    else:
        composable_node = ComposableNode(
                package="romea_localisation_imu_plugin",
                plugin="romea::IMULocalisationPlugin",
                name=plugin_name,
                parameters=[plugin_configuration],
            )

        load_component = LoadComposableNodes(
            composable_node_descriptions=[composable_node],
            target_container=component_container),

        return list(load_component)


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("plugin_name", default_value="imu_plugin"))

    declared_arguments.append(DeclareLaunchArgument("plugin_configuration_file_path"))

    declared_arguments.append(DeclareLaunchArgument("component_container", default_value=""))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
