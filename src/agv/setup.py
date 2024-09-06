from setuptools import find_packages, setup
import os
from glob import glob


package_name = "agv"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*launch.[pxy][yma]*")),
        ("share/" + package_name + "/description", glob("description/**")),
        ("share/" + package_name + "/config", glob("config/**")),
        ("share/" + package_name + "/worlds", glob("worlds/**")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xvshuo",
    maintainer_email="karmueo@163.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
