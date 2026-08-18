from glob import glob

from setuptools import find_packages, setup

package_name = "s10_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/mjcf", glob("mjcf/*.xml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="s10-perception-racing contributors",
    maintainer_email="89072276+bowenwan6@users.noreply.github.com",
    description="Simulated lidar, height map and ground-truth odometry for the S10 in MuJoCo.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sim_node = s10_perception.sim_node:main",
            "viewer_node = s10_perception.viewer_node:main",
        ],
    },
)
