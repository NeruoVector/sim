from glob import glob
import os

from setuptools import find_packages, setup

package_name = "nv_telemetry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NeruoVector",
    maintainer_email="a100822@naver.com",
    description="PX4 텔레메트리 요약 노드",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telemetry = nv_telemetry.telemetry_node:main",
        ],
    },
)
