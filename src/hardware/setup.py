from setuptools import find_packages, setup

package_name = "hardware"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mic",
    maintainer_email="misha.nekrasov@gmail.com",
    description="Communicates ROS2 messages and actions to arduino",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["hardware = hardware.hardware:main"],
    },
)
