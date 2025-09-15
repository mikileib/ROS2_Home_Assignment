from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'health_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Leib',
    maintainer_email='michael.leib98@gmail.com',
    description='Robot health monitoring system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_analyzer_node = health_monitor.health_analyzer_node:main',
            'alert_manager_node = health_monitor.alert_manager_node:main',
        ],
    },
)