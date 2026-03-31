from setuptools import setup

package_name = "stm32_firmware_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Flux",
    maintainer_email="flux@example.com",
    description="STM32 firmware serial bridge for forest rover",
    license="MIT",
    entry_points={
        "console_scripts": [
            "stm32_bridge_node = stm32_firmware_driver.bridge_node:main",
        ],
    },
)
