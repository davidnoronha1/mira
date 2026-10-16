from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mira2_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [path for path in glob('launch/*') if os.path.isfile(path)]),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'ardusub'), glob('config/ardusub/*.parm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='davidnoronha@outlook.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixhawk_controller = mira2_hardware.pixhawk_controller:main',
            'killswitch = mira2_hardware.killswitch:main',
        ],
    },
)
