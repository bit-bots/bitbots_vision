import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='true: activates simulation time, switches to simulation color settings and deactivates launching of an image provider'
        ),
        launch.actions.DeclareLaunchArgument(
            name='camera',
            default_value='true',
            description='true: launches an image provider to get images from a camera (unless sim:=true)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='basler',
            default_value='true',
            description='true: launches the basler camera driver instead of the wolves image provider'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dummyball',
            default_value='false',
            description='true: does not start the ball detection to save resources'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false',
            description='true: activates publishing of several debug images'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_game_settings',
            default_value='false',
            description='true: loads additional game settings'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskset',
            default_value='taskset -c 6,7'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskset',
            default_value=''
        ),
        launch_ros.actions.Node(
            package='bitbots_vision',
            executable='vision',
            name='bitbots_vision',
            output='screen',
            parameters=[
                {
                    'vision_publish_debug_image': launch.substitutions.LaunchConfiguration('debug')
                },
                {
                    'vision_publish_HSV_mask_image': launch.substitutions.LaunchConfiguration('debug')
                },
                {
                    'vision_publish_field_mask_image': launch.substitutions.LaunchConfiguration('debug')
                },
                {
                    'neural_network_type': 'dummy'
                },
                get_package_share_directory(
                    'bitbots_vision') + '/config/visionparams.yaml',
                get_package_share_directory(
                    'bitbots_vision') + '/config/simparams.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
