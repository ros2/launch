#!/usr/bin/env python

import glob

from setuptools import setup

package_name = 'apex_launchtest'

setup(
    name=package_name,
    version='0.1',
    description='Apex integration test runner and utilities',

    author='Pete Baughman',
    author_email='pete.baughman@apex.ai',

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/apex_launchtest']),
        ('lib/' + package_name, glob.glob('example_processes/**')),
        ('share/' + package_name + '/examples', glob.glob('examples/[!_]**')),
        ('bin', ['scripts/apex_launchtest']),
    ],
    packages=[
        'apex_launchtest',
        'apex_launchtest.asserts',
        'apex_launchtest.event_handlers',
        'apex_launchtest.util',
    ],
    tests_require=['pytest'],
    zip_safe=True,
)
