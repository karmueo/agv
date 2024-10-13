"""
Author: 沈昌力
Date: 2024-10-09 16:50:52
LastEditTime: 2024-10-09 17:47:52
LastEditors: 沈昌力
Description: 
FilePath: /agv/src/bcr_bot_patrol/setup.py
"""

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "bcr_bot_patrol"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            ["config/patrol_config.yaml"],
        ),  # 将 config/patrol_config.yaml 文件复制到安装目录中的 share/<package_name>/config 目录下。
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),  # 将所有位于 launch 目录下的以 .launch.py 结尾的文件复制到安装目录中的 share/<package_name>/launch 目录下
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xvshuo",
    maintainer_email="karmueo@163.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_node=bcr_bot_patrol.patrol_node:main",
            "speaker=bcr_bot_patrol.speaker:main",
            "param_test=bcr_bot_patrol.param_test:main",
        ],
    },
)
