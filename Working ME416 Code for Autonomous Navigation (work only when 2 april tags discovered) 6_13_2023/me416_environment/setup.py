from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    scripts=['scripts/me416_environment/motor_command_model.py'],
    packages=['me416_environment'],
    package_dir={'': 'scripts'}
)

setup(**setup_args)
