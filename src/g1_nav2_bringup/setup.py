from glob import glob
from setuptools import setup

package_name = "g1_nav2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu22",
    maintainer_email="xu22@example.com",
    description="Nav2 bringup helpers for Unitree G1 using FAST-LIO odometry and Mid360 lidar.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "g1_auto_global_localization = "
            "g1_nav2_bringup.auto_global_localization:main",
        ],
    },
)
