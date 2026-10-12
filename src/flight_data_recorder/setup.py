from setuptools import find_packages, setup

package_name = "flight_data_recorder"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="ljz",
    maintainer_email="ljz@example.com",
    description="D题完整飞行数据自动记录器",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "flight_data_recorder_node = flight_data_recorder.recorder_node:main",
        ],
    },
)
