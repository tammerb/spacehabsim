#!/usr/bin/env python3

"""
Example showing how to build a world and use it with pyrobosim,
additionally starting up a ROS interface.
"""
import os
import rclpy
import threading
import numpy as np

from pyrobosim.core import Robot, World, WorldYamlLoader
#from pyrobosim.gui import start_gui
from spacehabsim.gui.main import start_gui
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
# from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper
from pyrobosim.navigation import OccupancyGrid


def get_data_folder(): # TODO: Move this into a utils folder
    """
    Get a path to the folder containing data.

    If running using ``ros2 run``, this looks at the installed data in the
    ``pyrobosim`` colcon package's share directory.

    If running standalone, this looks at the data folder in the actual source.

    :return: Path to data folder.
    :rtype: str
    """
    try:
        # If running as a ROS 2 node, get the data folder from the package share directory.
        from ament_index_python.packages import get_package_share_directory

        data_folder = os.path.join(get_package_share_directory("spacehabsim"), "data")
    except:
        # Else, assume it's relative to the file's current directory.
        data_folder = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "data"
        )

    return data_folder


data_folder = get_data_folder()


def create_world():
    """Create a space habitat world"""
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "location_data.yaml"),
        objects=os.path.join(data_folder, "object_data.yaml"),
    )

    # Add rooms
    r1coords = [(-3.0, -1.0),
                (-3.0, 1.0),
                (3.0, 1.0),
                (3.0, -1.0)]
    world.add_room(name="module", footprint=r1coords, color=[0, 0, 0])

    # Add hallways between the rooms
    """
    world.add_hallway(room_start="", room_end="", width=)
    world.add_hallway(
        room_start="b",
        room_end="",
        width=,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
    )
    """

    # Add locations
    rack_a = world.add_location(
       name="rack_a", category="storage_rack", parent="module", pose=Pose(x=0, y=0.675, yaw=0)
    )
    rack_b = world.add_location(
       name="rack_b", category="storage_rack", parent="module", pose=Pose(x=-2.65, y=0, yaw=1.57)
    )
    # printer = world.add_location(
    #    category="printer", parent="module", pose=Pose(x=0, y=0.74, yaw=1.57)
    # )
    # eclss = world.add_location(
    #    category="eclss", parent="module", pose=Pose(x=2, y=0.74, yaw=1.57)
    # )

    ## Add objects
    world.add_object(
        name="box_1", category="bracket", parent=rack_b
    )
    world.add_object(
        name="box_2", category="bracket", parent=rack_b
    )
    world.add_object(
        name="box_3", category="bracket", parent=rack_b
    )
    world.add_object(
        name="box_4", category="bracket", parent=rack_b
    )


    # Add a robot
    robot = Robot(
        name="robot",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(),
    )
    world.add_robot(robot, loc="module")

    planner_config = {
        "grid": OccupancyGrid.from_world(
            world, resolution=0.05, inflation_radius=1.5 * robot.radius
        ),
        "diagonal_motion": True,
        "heuristic": "euclidean",
        "compress_path": False,
    }

    path_planner = PathPlanner("astar", **planner_config)
    robot.set_path_planner(path_planner)


    return world


def create_world_from_yaml(world_file):
    return WorldYamlLoader().from_yaml(os.path.join(data_folder, world_file))


def main():
    """Initializes ROS node"""
    rclpy.init()

    node = WorldROSWrapper(state_pub_rate=0.1, dynamics_rate=0.01)
    node.declare_parameter("world_file", value="")

    # Set the world
    world_file = node.get_parameter("world_file").get_parameter_value().string_value
    if world_file == "":
        node.get_logger().info("Creating demo world programmatically.")
        world = create_world()
    else:
        node.get_logger().info(f"Using world file {world_file}.")
        world = create_world_from_yaml(world_file)

    node.set_world(world)


    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)

    return


if __name__ == "__main__":
    node = main()
