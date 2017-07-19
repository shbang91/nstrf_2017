from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['button_gui', 'teleop'],
    package_dir={'': 'src'}
)

setup(**d)