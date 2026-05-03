from setuptools import setup

package_name = 'xarm_pick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darklord',
    maintainer_email='25314403062@qq.com',
    description='xArm 1S pick state machine.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick = xarm_pick.pick_node:main',
            'calibrate_homography = xarm_pick.calibrate_homography:main',
            'pick_2d = xarm_pick.pick_2d:main',
        ],
    },
)
