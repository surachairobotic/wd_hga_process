from setuptools import setup
import subprocess, os, platform
from glob import glob

package_name = 'wd_hga_process'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Surachai Rodsai',
    maintainer_email='surachairobotic@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wd_ur = wd_hga_process.wd_ur:main',
            'wd_mir = wd_hga_process.wd_mir:main',
            'wd_webserver = wd_hga_process.wd_webserver:main'
        ],
    },
)
