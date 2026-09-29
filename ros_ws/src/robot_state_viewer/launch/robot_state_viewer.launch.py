from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_state_viewer")
    default_rviz_config = os.path.join(pkg_share, "rviz", "rviz_config.rviz")
    default_urdf = os.path.join(pkg_share, "urdf", "mors.urdf")
    default_heightmap_config = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "..", "config", "heightmap_builder.yaml")
    )

    with open(default_urdf, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to RViz config file",
    )
    heightmap_config_arg = DeclareLaunchArgument(
        "heightmap_config_path",
        default_value=default_heightmap_config,
        description="Path to heightmap_builder.yaml used to decode HEIGHTMAP",
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame_id",
        default_value="world",
        description="Global frame used for TF root and pointcloud frame_id",
    )
    pointcloud_frame_arg = DeclareLaunchArgument(
        "pointcloud_frame_id",
        default_value=LaunchConfiguration("world_frame_id"),
        description="Frame id assigned to /camera/poincloud",
    )
    base_link_frame_arg = DeclareLaunchArgument(
        "base_link_frame_id",
        default_value="base_link",
        description="Robot base link frame id",
    )

    viewer_node = Node(
        package="robot_state_viewer",
        executable="robot_state_viewer_node",
        name="robot_state_viewer",
        output="screen",
        parameters=[
            {
                "world_frame_id": LaunchConfiguration("world_frame_id"),
                "pointcloud_frame_id": LaunchConfiguration("pointcloud_frame_id"),
                "base_link_frame_id": LaunchConfiguration("base_link_frame_id"),
                "heightmap_config_path": LaunchConfiguration("heightmap_config_path"),
            }
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", LaunchConfiguration("rviz_config")],
    # )

    ld = LaunchDescription()
    ld.add_action(rviz_config_arg)
    ld.add_action(heightmap_config_arg)
    ld.add_action(world_frame_arg)
    ld.add_action(pointcloud_frame_arg)
    ld.add_action(base_link_frame_arg)
    ld.add_action(viewer_node)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(rviz_node)
    return ld
