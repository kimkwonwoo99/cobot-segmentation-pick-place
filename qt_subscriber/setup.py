## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/qt_subscriber/poppy_window.py', 'src/qt_subscriber/put_window.py', 'src/qt_subscriber/pick_window.py'],
    packages=['qt_subscriber'],
    package_dir={'': 'src'}
)

setup(**setup_args)
