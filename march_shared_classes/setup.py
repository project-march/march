# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from setuptools import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['march_shared_classes', 'march_shared_classes.gait', 'march_shared_classes.exceptions'],
)

setup(**setup_args)
