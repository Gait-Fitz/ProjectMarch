# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=["dynamic_reconfigure_parameters"],
    package_dir={"": "src"},
    scripts=["src/dynamic_reconfigure_parameters/march_dynamic_reconfigure_node"],
)

setup(**setup_args)
