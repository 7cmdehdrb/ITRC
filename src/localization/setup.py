import sys
import os
from setuptools import find_packages, setup

package_name = "localization"


resource_path = os.path.join(os.path.dirname(__file__), "resource")
fils_names = os.listdir(resource_path)
filtered_files = [f for f in fils_names if f.endswith((".json", ".xml", ".yaml"))]


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name] + [f"resource/{f}" for f in filtered_files],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="min",
    maintainer_email="7cmdehdrb@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [],
    },
)
