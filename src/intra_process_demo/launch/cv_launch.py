# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import unittest

import launch
import launch_ros.actions



def generate_launch_description():
    launch_description = launch.LaunchDescription()

    image_pipeline_all_in_one = launch_ros.actions.Node(
        executable="image_pipeline_all_in_one",
        name="image_pipeline_all_in_one",
        package="intra_process_demo",
    )
    launch_description.add_action(image_pipeline_all_in_one)
    edge_detection = launch_ros.actions.Node(
        executable="edge_detector_node",
        name="edge_detector_node",
        package="intra_process_demo",
    )
    launch_description.add_action(edge_detection)
    dense_optical_flow = launch_ros.actions.Node(
        executable="dense_optical_flow_node",
        name="dense_optical_flow_node",
        package="intra_process_demo",
    )
    launch_description.add_action(dense_optical_flow)
    dense_optical_flow_2 = launch_ros.actions.Node(
        executable="dense_optical_flow_2_node",
        name="dense_optical_flow_2_node",
        package="intra_process_demo",
    )
    launch_description.add_action(dense_optical_flow_2)

    return launch_description
