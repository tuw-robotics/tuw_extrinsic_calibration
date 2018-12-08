#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tuw_extrinsic_camera'],
    package_dir={'': 'src'},
    scripts=['scripts/tuw_extrinsic_camera']
)

setup(**d)
