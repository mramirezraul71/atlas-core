from setuptools import setup

package_name = "atlas_supervisor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul",
    maintainer_email="raul@atlas-robot.dev",
    description="Atlas supervisor: system orchestration, governance, heartbeat",
    license="MIT",
    entry_points={
        "console_scripts": [
            "supervisor_node = atlas_supervisor.supervisor_node:main",
        ],
    },
)
