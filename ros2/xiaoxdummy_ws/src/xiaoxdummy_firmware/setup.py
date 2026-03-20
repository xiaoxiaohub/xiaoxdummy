from glob import glob
import os

from setuptools import setup

package_name = 'xiaoxdummy_firmware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiehuang',
    maintainer_email='hydrangeahuman@gmail.com',
    description='ROS 2 firmware bridge for the Xiaoxdummy arm.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xiaoxdummy_firmware_server = xiaoxdummy_firmware.xiaoxdummy_firmware_server:main',
        ],
    },
)
