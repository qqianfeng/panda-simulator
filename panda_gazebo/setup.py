#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['panda_gazebo'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'sensor_msgs']
)
setup(**setup_args)
