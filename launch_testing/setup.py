import glob

from setuptools import find_packages
from setuptools import setup


setup(
    name='launch_testing',
    version='0.7.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/launch_testing']),
        ('lib/launch_testing', glob.glob('example_processes/**')),
        ('share/launch_testing/examples', glob.glob('examples/[!_]**')),
        ('bin', ['scripts/launchtest']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Pete Baughman',
    author_email='pete.baughman@apex.ai',
    maintainer='Pete Baughman',
    maintainer_email='pete.baughman@apex.ai',
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Test the output of a process.',
    long_description='ROS launch testing suite.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
