from setuptools import setup

package_name = "atlas_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raul",
    maintainer_email="raul@atlas-robot.dev",
    description="Atlas planning: task decomposition and path planning",
    license="MIT",
    entry_points={
        "console_scripts": [
            "task_planner = atlas_planning.task_planner:main",
            "path_planner = atlas_planning.path_planner:main",
        ],
    },
)
