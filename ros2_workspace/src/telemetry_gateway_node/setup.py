from setuptools import find_packages, setup

package_name = "telemetry_gateway_node"

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
    description="LoRa telemetry heartbeat gateway for forest rover",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telemetry_gateway = telemetry_gateway_node.telemetry_gateway_main:main",
        ],
    },
)
