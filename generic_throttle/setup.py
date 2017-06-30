#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   ##  don't do this unless you want a globally visible script
   packages=['generic_throttle'],
   package_dir={'': 'src'}
)

setup(**d)
