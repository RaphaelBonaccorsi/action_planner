# Copyright 2019 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare launch arguments
    domain_arg = DeclareLaunchArgument('domain_file', default_value='/pddl/domain.pddl', description='The PDDL domain file to use.')
    problem_arg = DeclareLaunchArgument('problem_file', default_value='/pddl/problem.pddl', description='The PDDL problem file to use.')
    # Use LaunchConfiguration to get the values of arguments

    domain_file_value = LaunchConfiguration('domain_file')
    problem_file_value = LaunchConfiguration('problem_file')

    ld = LaunchDescription()

    nodes_to_add = [
        Node(
            package='action_planner',
            executable='lifecycle_manager.py',
            name='lifecycle_manager',
            output='screen',
            parameters=[]
        ),
        Node(
            package='action_planner',
            executable='action_planner.py',
            name='action_planner',
            output='screen',
            parameters=[{"pddl_domain": domain_file_value}, {"pddl_problem": problem_file_value}]
        ),
        Node(
            package='action_planner',
            executable='mission_controller.py',
            name='mission_controller',
            output='screen',
            parameters=[]
        ),
        Node(
            package='action_planner',
            executable='move_acima.py',
            name='move_acima',
            output='screen',
            parameters=[]
        ),
        Node(
            package='action_planner',
            executable='move_abaixo.py',
            name='move_abaixo',
            output='screen',
            parameters=[]
        ),
        Node(
            package='action_planner',
            executable='entrar.py',
            name='entrar',
            output='screen',
            parameters=[]
        ),
        Node(
            package='action_planner',
            executable='sair.py',
            name='sair',
            output='screen',
            parameters=[]
        ),
    ]

    # Create the launch description and populate
    ld.add_action(domain_arg)
    ld.add_action(problem_arg)

    for node in nodes_to_add:
        ld.add_action(node)

    return ld