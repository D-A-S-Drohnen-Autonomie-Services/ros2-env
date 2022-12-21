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
    publisher_node_parameters = {
        'reliability': 'reliable',
        'show_camera': False,
        'burger_mode': True,
        'frequency': 5.0,
    }
    subscriber_node_parameters = {
        'reliability': 'reliable',
        'show_image': False,
    }

    # Launch the process that will receive the images.
    # This is the process that gets to decide when to tear the launcher down.
    # It will exit when the regex for receiving images is matched.
    showimage_executable = 'showimage'
    showimage_name = 'showimage'

    showimage_node = launch_ros.actions.Node(
        executable=showimage_executable,
        name=showimage_name,
        package="image_tools",
        parameters=[subscriber_node_parameters],
    )
    launch_description.add_action(showimage_node)

    # Launch the process that will publish the images.
    # This process will be exited when the launcher is torn down.
    cam2image_executable = 'cam2image'
    cam2image_name = 'cam2image'

    cam2image_node = launch_ros.actions.Node(
        executable=cam2image_executable,
        name=cam2image_name,
        package="image_tools",
        parameters=[publisher_node_parameters],
    )
    launch_description.add_action(cam2image_node)

    return launch_description
