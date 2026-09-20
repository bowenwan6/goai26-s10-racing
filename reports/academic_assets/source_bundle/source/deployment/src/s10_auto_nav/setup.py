from setuptools import find_packages, setup

package_name = "s10_auto_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="s10-perception-racing contributors",
    maintainer_email="wang.bowen@icloud.com",
    description="Pure-pursuit waypoint following for the S10 racing course.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "waypoint_follower = s10_auto_nav.follower_node:main",
            "strategy_router = s10_auto_nav.strategy_router_node:main",
            "segment_recorder = s10_auto_nav.segment_recorder:main",
            "pitfail_recorder = s10_auto_nav.pitfail_recorder:main",
        ],
    },
)
