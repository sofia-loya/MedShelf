import os
from glob import glob
from setuptools import find_packages, setup

package_name = "my_arm_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="MedShelf perception nodes for color and shelf-slot detection.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bin_detector = my_arm_perception.bin_detector:main",
            "shelf_test_publisher = my_arm_perception.shelf_test_publisher:main",
	    "task_manager = my_arm_perception.task_manager:main",
	    "mock_manipulation = my_arm_perception.mock_manipulation:main",
        ],
    },
)
