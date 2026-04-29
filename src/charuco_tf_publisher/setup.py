from setuptools import setup
from glob import glob

package_name = 'charuco_tf_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darklord',
    maintainer_email='limjy09130@gmail.com',
    description='ChArUco board detector that broadcasts a TF for easy_handeye2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'charuco_tf_node = charuco_tf_publisher.charuco_tf_node:main',
        ],
    },
)
