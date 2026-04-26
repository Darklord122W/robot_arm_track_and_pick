import os
from glob import glob
from setuptools import setup

package_name = 'cube_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darklord',
    maintainer_email='25314403062@qq.com',
    description='Depth + edge fusion cube detector for the xArm workspace.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cube_detector_node = cube_detector.detector_node:main',
        ],
    },
)
