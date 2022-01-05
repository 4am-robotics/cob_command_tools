#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['cob_auto_tools'],
    package_dir={'': 'src'}
)

setup(**d)