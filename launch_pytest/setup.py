import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'launch_pytest'

setup(
    name=package_name,
    version='0.19.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'lib/{package_name}', glob.glob('example_processes/**')),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/examples', glob.glob(f'test/{package_name}/examples/[!_]**')),
    ],
    entry_points={
        'pytest11': [
            'launch_pytest = launch_pytest.plugin'
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ivan Paunovic',
    author_email='ivanpauno@ekumenlabs.com',
    maintainer='William Woodall, Michel Hidalgo',
    maintainer_email='william@osrfoundation.org, michel@ekumenlabs.com',
    url='https://github.com/ros2/launch',
    download_url='https://github.com/ros2/launch/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Create tests which involve launch files and multiple processes.',
    long_description=('A package to create tests which involve'
                      ' launch files and multiple processes.'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
