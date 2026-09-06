# Copyright (c) 2026 Okan Demir
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

"""Bringup for the docking tests."""

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import LaunchConfigAsBool

# Nav2
USE_SIM_TIME = "True"
AUTOSTART = "True"
USE_COMPOSITION = "False"
USE_RESPAWN = "False"
LOG_LEVEL = "info"
USE_RVIZ = "True"

# Must match amcl's seeded initial_pose in the parameter file.
SPAWN_X, SPAWN_Y, SPAWN_Z = "-2.00", "-0.50", "0.01"
SPAWN_ROLL, SPAWN_PITCH, SPAWN_YAW = "0.00", "0.00", "0.00"
ROBOT_NAME = "turtlebot3_waffle"

# Synthetic dock detection
# Ground truth must match home_dock in the parameter file.
DOCK_X, DOCK_Y, DOCK_YAW = 2.0, -0.6, 0.0
DOCK_DETECTION_RATE = 20.0
# Gaussian noise added to each published detection.
DOCK_NOISE_STDDEV_X = 0.0  # [m]
DOCK_NOISE_STDDEV_Y = 0.0  # [m]
DOCK_NOISE_STDDEV_YAW = 0.0  # [rad]
DOCK_NOISE_SEED = 0  # 0 is nondeterministic; fix it to replay a disturbance


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory("nav2_dock_tests")
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")

    map_yaml_file = os.path.join(pkg_dir, "maps", "dock_test_world.yaml")
    params_file = os.path.join(pkg_dir, "params", "nav2_dock_tests_params.yaml")
    rviz_config_file = os.path.join(pkg_dir, "rviz", "nav2_dock_tests_view.rviz")
    world = os.path.join(pkg_dir, "worlds", "dock_test_world.sdf.xacro")
    robot_sdf = os.path.join(sim_dir, "urdf", "gz_waffle.sdf.xacro")

    headless = LaunchConfigAsBool("headless")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value="True",
        description="Run Gazebo without its GUI",
    )

    # This package's own models take precedence, so a dock model dropped into models/ can
    # shadow or extend what nav2_minimal_tb3_sim ships.
    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(pkg_dir, "models")
    )
    set_env_vars_resources_sim = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "models")
    )
    set_env_vars_resources_sim_parent = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", str(Path(sim_dir).parent.resolve())
    )

    urdf = os.path.join(sim_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf, "r") as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        remappings=remappings,
    )

    world_sdf = tempfile.mktemp(prefix="nav2_dock_tests_", suffix=".sdf")
    world_sdf_xacro = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world]
    )

    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_sdf],
        output="screen",
    )

    # gz must not start until xacro has finished writing world_sdf
    start_gazebo_after_xacro = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=world_sdf_xacro, on_exit=[gazebo_server]
        )
    )

    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression(["not ", headless])),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, "launch", "spawn_tb3.launch.py")
        ),
        launch_arguments={
            "use_sim_time": USE_SIM_TIME,
            "robot_name": ROBOT_NAME,
            "robot_sdf": robot_sdf,
            "x_pose": SPAWN_X,
            "y_pose": SPAWN_Y,
            "z_pose": SPAWN_Z,
            "roll": SPAWN_ROLL,
            "pitch": SPAWN_PITCH,
            "yaw": SPAWN_YAW,
        }.items(),
    )

    # Stands in for a perception pipeline: SimpleChargingDock::getRefinedPose subscribes to
    dock_pose_publisher_cmd = Node(
        package="nav2_dock_tests",
        executable="dock_pose_publisher.py",
        name="dock_pose_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "frame_id": "map",
                "dock_x": DOCK_X,
                "dock_y": DOCK_Y,
                "dock_yaw": DOCK_YAW,
                "publish_rate": DOCK_DETECTION_RATE,
                "noise_stddev_x": DOCK_NOISE_STDDEV_X,
                "noise_stddev_y": DOCK_NOISE_STDDEV_Y,
                "noise_stddev_yaw": DOCK_NOISE_STDDEV_YAW,
                "noise_seed": DOCK_NOISE_SEED,
            }
        ],
        remappings=remappings,
    )

    # slam:=False selects AMCL over slam_toolbox inside bringup_launch.py
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "map": map_yaml_file,
            "use_sim_time": USE_SIM_TIME,
            "params_file": params_file,
            "autostart": AUTOSTART,
            "use_composition": USE_COMPOSITION,
            "use_respawn": USE_RESPAWN,
            "use_localization": "True",
            "use_keepout_zones": "False",
            "use_speed_zones": "False",
            "log_level": LOG_LEVEL,
            "container_name": "nav2_container",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "rviz_launch.py")),
        condition=IfCondition(USE_RVIZ),
        launch_arguments={
            "use_sim_time": USE_SIM_TIME,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_headless_cmd)

    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources_sim)
    ld.add_action(set_env_vars_resources_sim_parent)

    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(start_gazebo_after_xacro)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)
    ld.add_action(start_robot_state_publisher_cmd)

    ld.add_action(dock_pose_publisher_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
