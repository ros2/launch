from setuptools import find_packages
from setuptools import setup

setup(
    name='launch_xml',
    version='0.7.3',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ivan Paunovic',
    author_email='ivanpauno@ekumenlabs.com',
    maintainer='Ivan Paunovic',
    maintainer_email='ivanpauno@ekumenlabs.com',
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Launch process from different front-ends.',
    long_description=(
        'This package provides the to parse different front-ends '
        'to a programmatic launch process.'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
