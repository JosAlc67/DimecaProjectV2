import os
from glob import glob

from setuptools import find_packages, setup

package_name = "irb2600_coating_cell"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.STL") + glob("meshes/*.stl")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DIMECA Group 1",
    maintainer_email="manuel.alcivar12@gmail.com",
    description=(
        "Simulated coating cell: planning-scene setup, simulated RGB-D "
        "perception, and spray_on logical control for the DIMECA digital "
        "prototype."
    ),
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scene_setup_node = irb2600_coating_cell.scene_setup_node:main",
            "perception_sim_node = irb2600_coating_cell.perception_sim_node:main",
            "spray_controller_node = irb2600_coating_cell.spray_controller_node:main",
            "trajectory_planner_node = irb2600_coating_cell.trajectory_planner_node:main",
            "replanning_executor_node = irb2600_coating_cell.replanning_executor_node:main",
            "go_home_node = irb2600_coating_cell.go_home_node:main",
            "gui_control_node = irb2600_coating_cell.gui_control_node:main",
        ],
    },
)
