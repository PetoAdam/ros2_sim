from setuptools import find_packages, setup

package_name = "ros2_sim_pid_tuner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/pid_tuner.launch.py"]),
        (f"share/{package_name}/config", ["config/pid_tuner.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adam Peto",
    maintainer_email="peto.adam@yahoo.com",
    description="PID tuning node for ros2_sim",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros2_sim_pid_tuner_node = ros2_sim_pid_tuner.tuner_node:main",
        ],
    },
)
