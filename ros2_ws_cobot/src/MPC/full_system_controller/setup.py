from setuptools import find_packages, setup
from glob import glob
import os

package_name = "full_system_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "configs"), glob("configs/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andre-rebelo-teixeira",
    maintainer_email="andre.r.teixeira@tecnico.ulisboa.pt",
    description="Full System controller for the space cobot vehicle",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "full_system_controller = full_system_controller.full_system_controller:main"
        ],
    },
)
