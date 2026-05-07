from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['navigation_pkg'],
    package_dir={'': 'src'},
    scripts=['scripts/aaaVFH.py', 'scripts/robot_visualizer.py', 'scripts/VFHtest.py'],
)

setup(**d)