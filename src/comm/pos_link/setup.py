from glob import glob
import os

from setuptools import setup

package_name = "pos_link"

setup(
    name=package_name,
    version="1.2.0",
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
    description="ESP32-S3 坐标链路：TCP 定长帧 → /esp32/position",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = pos_link.bridge_node:main",
        ],
    },
)
