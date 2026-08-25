import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rov2_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROV2 Maintainers",
    maintainer_email="dev@example.com",
    description="Sim-only validation tooling for the ROV2 framework (M2 sim_pilot + bringup).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_pilot = rov2_sim.sim_pilot:main",
        ],
    },
)
