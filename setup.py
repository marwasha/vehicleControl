#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['vehicleControl'],
    package_dir={'': 'include'},
)

setup(**setup_args)
