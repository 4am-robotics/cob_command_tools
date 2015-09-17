#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   ##  don't do this unless you want a globally visible script
   # scripts=['bin/myscript'],
   #scripts=['src/simple_script_server.py'],
   #py_modules=['cob_script_server'],
   packages=['simple_script_server'],
   package_dir={'': 'src'}
)

setup(**d)