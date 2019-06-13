from setuptools import find_packages
from setuptools import setup

setup(
    name='launch_frontend',
    version='0.8.1',
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
    description='Frontend extensions for the `launch` package.',
    long_description=(
        'This package provides front-end extensions for the `launch` package.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
