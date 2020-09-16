# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['march_gait_selection', 'march_gait_selection.dynamic_gaits', 'march_gait_selection.state_machine'],
    package_dir={'': 'src'},
    scripts=['scripts/march_gait_selection_node'],
)

setup(**setup_args)
