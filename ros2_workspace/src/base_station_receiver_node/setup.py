from setuptools import find_packages, setup

package_name = "base_station_receiver_node"

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
    maintainer="Forest Rover Team",
    maintainer_email="dev@example.com",
    description="Base station telemetry receiver and logging for forest rover LoRa link",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_station_receiver = base_station_receiver_node.base_station_receiver_main:main",
        ],
    },
)
