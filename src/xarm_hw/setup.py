from setuptools import setup
import os
from glob import glob

package_name = 'xarm_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darklord',
    maintainer_email='you@example.com',
    description='xArm hardware driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'xarm_hw_driver = xarm_hw.driver:main',
        ],
    },
)
