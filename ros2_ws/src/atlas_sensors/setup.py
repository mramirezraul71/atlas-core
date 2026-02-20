from setuptools import setup

package_name = "atlas_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul",
    maintainer_email="raul@atlas-robot.dev",
    description="Atlas hardware abstraction layer: sensors and joint state publishing",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_publisher = atlas_sensors.imu_publisher:main",
            "joint_state_publisher = atlas_sensors.joint_state_publisher:main",
            "force_torque_publisher = atlas_sensors.force_torque_publisher:main",
        ],
    },
)
