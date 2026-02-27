from setuptools import setup

package_name = "atlas_motor_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul",
    maintainer_email="raul@atlas-robot.dev",
    description="Atlas motor control: balance, gait, and joint command dispatch",
    license="MIT",
    entry_points={
        "console_scripts": [
            "balance_controller = atlas_motor_control.balance_controller:main",
            "gait_generator = atlas_motor_control.gait_generator:main",
            "joint_commander = atlas_motor_control.joint_commander:main",
        ],
    },
)
