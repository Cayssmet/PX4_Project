from glob import glob
import os

from setuptools import setup

package_name = "px4_offboard"

setup(
    name=package_name,
    version="3.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cayssmet",
    maintainer_email="calxn29@qq.com",
    description="树莓派 ↔ PX4 的 MAVLink 桥（零依赖编解码 + 坐标换算 + 只读状态节点）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_node = px4_offboard.px4_node:main",
        ],
    },
)
