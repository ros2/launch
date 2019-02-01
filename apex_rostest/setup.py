#!/usr/bin/env python

from setuptools import setup
import glob

package_name = 'apex_rostest'

setup(
    name=package_name,
    version='0.1',
    description='Apex integration test runner and utilities',

    author='Pete Baughman',
    author_email='pete.baughman@apex.ai',

    data_files=[
        ('lib/' + package_name, glob.glob('example_nodes/**')),
        ('share/' + package_name + '/examples', glob.glob('examples/**')),
        ('bin', ['scripts/apex_rostest']),
    ],
    packages=[
        'apex_rostest',
        'apex_rostest.asserts',
        'apex_rostest.event_handlers',
        'apex_rostest.util',
    ],
    tests_require=["pytest"],
    zip_safe=True,
)
