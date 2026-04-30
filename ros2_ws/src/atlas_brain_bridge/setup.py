from setuptools import setup

package_name = "atlas_brain_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul",
    maintainer_email="raul@atlas-robot.dev",
    description="Atlas brain bridge: ROS 2 to existing Atlas AI/ANS/memory",
    license="MIT",
    entry_points={
        "console_scripts": [
            "brain_bridge_node = atlas_brain_bridge.brain_bridge_node:main",
        ],
    },
)
