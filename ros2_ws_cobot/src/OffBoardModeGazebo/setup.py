# OffBoardModeGazebo/setup.py

from setuptools import setup
import os
from glob import glob

package_name = "OffBoardModeGazebo"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andre Teixeira",
    maintainer_email="andre.rebelo.teixeira@hotmail.com",
    description="Package for controlling a PX4 SITL in OffBoard mode within Gazebo",
    license="Insert your license here",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "OffBoardModeGazebo = OffBoardModeGazebo.OffBoardModeGazebo:main"
        ],
    },
)
