from glob import glob
import os

from setuptools import find_packages, setup


package_name = "motion_recorder_gui"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "static"), glob("static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Garcia",
    maintainer_email="garcia.miguel.onate@gmail.com",
    description="Web GUI for recording and replaying G1 joint motions.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "motion_recorder_gui = motion_recorder_gui.server:main",
        ],
    },
)
