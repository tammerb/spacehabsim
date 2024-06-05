import os

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