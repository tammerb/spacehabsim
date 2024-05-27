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
from gui.space_gui import start_my_gui
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
# from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper

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

    # Set the location and object metadata (TODO: Make new folders and files)
    world.set_metadata(
        locations=os.path.join(data_folder, "location_data.yaml"),
        objects=os.path.join(data_folder, "object_data.yaml"),
    )

    # Add rooms
    r1coords = [(-3.0, 0.5),
                (-2.5, 1.0),
                (-1.5, 1.0),
                (-1.0, 0.5),
                (-1.0, -0.5),
                (-1.5, -1.0),
                (-2.5, -1.0),
                (-3.0, -0.5)]
    world.add_room(name="module_5", footprint=r1coords, color=[1, 0, 0])
    r2coords = [(3.0, 0.5),
                (2.5, 1.0),
                (1.5, 1.0),
                (1.0, 0.5),
                (1.0, -0.5),
                (1.5, -1.0),
                (2.5, -1.0),
                (3.0, -0.5)]
    world.add_room(name="module_2", footprint=r2coords, color=[0, 0.6, 0])
    #r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    #world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])

    # Add hallways between the rooms
    world.add_hallway(room_start="module_5", room_end="module_2", width=0.7)
    #world.add_hallway(
    #    room_start="bathroom",
    #    room_end="bedroom",
    #    width=0.5,
    #    conn_method="angle",
    #    conn_angle=0,
    #    offset=0.8,
    #)
    #world.add_hallway(
    #    room_start="kitchen",
    #    room_end="bedroom",
    #    width=0.6,
    #    conn_method="points",
    #    conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
    #)

    # Add locations
    rack1 = world.add_location(
       category="rack", parent="module_5", pose=Pose(x=-2.75, y=0, yaw=0.0)
    )
    rack2 = world.add_location(
       category="rack", parent="module_2", pose=Pose(x=2, y=0.74, yaw=1.57)
    )
    #desk = world.add_location(
    #    category="desk", parent="bedroom", pose=Pose(x=3.15, y=3.65, yaw=0.0)
    #)
    #counter = world.add_location(
    #    category="counter",
    #    parent="bathroom",
    #    pose=Pose(x=-2.45, y=2.5, yaw=np.pi / 2.0 + np.pi / 16.0),
    #)

    ## Add objects
    world.add_object(
        category="wrench", parent=rack1, pose=Pose(x=-2.75, y=0.05, yaw=np.pi / 4.0)
    )
    world.add_object(
        category="tape", parent=rack2, pose=Pose(x=2, y=0.74, yaw=np.pi / 4.0)
    )
    #world.add_object(category="apple", parent=desk, pose=Pose(x=3.2, y=3.5, yaw=0.0))
    #world.add_object(category="apple", parent=table)
    #world.add_object(category="apple", parent=table)
    #world.add_object(category="water", parent=counter)
    #world.add_object(category="banana", parent=counter)
    #world.add_object(category="water", parent=desk)

    # Add a robot
    # Create path planner
    planner_config = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    path_planner = PathPlanner("rrt", **planner_config)
    robot = Robot(
        name="robot",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="module_5")

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
    start_my_gui(node.world)

    return


if __name__ == "__main__":
    node = main()
