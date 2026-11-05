from setuptools import find_packages, setup

package_name = "rm_flatness_nav"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Muhammad Qaisar Ali",
    maintainer_email="muhammadqaisarali1@gmail.com",
    description="RoboMaster AprilTag-guided cubic trajectory pursuit",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "apriltag_detector = rm_flatness_nav.apriltag_detector:main",
            "planner = rm_flatness_nav.planner:main",
        ],
    },
)
