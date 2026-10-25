from setuptools import find_packages, setup

package_name = "computer_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "train"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/config", ["config/camera_info.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mic",
    maintainer_email="misha.nekrasov@gmail.com",
    description="Camera-based detection of chess pieces, chessboard, etc",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["board_detection = computer_vision.board_detection:main"],
    },
)
