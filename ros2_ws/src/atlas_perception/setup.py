from setuptools import setup

package_name = "atlas_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul",
    maintainer_email="raul@atlas-robot.dev",
    description="Atlas perception: vision, object detection, SLAM",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_node = atlas_perception.vision_node:main",
            "object_detector = atlas_perception.object_detector:main",
            "slam_node = atlas_perception.slam_node:main",
        ],
    },
)
