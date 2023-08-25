import glob
from pathlib import Path

from setuptools import find_packages
from setuptools import setup


package_name = 'launch_pytest'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (
            f'share/{package_name}/examples',
            [x for x in glob.glob(
                f'test/{package_name}/examples/[!_]*') if Path(x).is_file()]
        ),
        (
            f'share/{package_name}/examples/executables',
            glob.glob(f'test/{package_name}/examples/executables/[!_]*')
        ),
    ],
    entry_points={
        'pytest11': ['launch_pytest = launch_pytest.plugin'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ivan Paunovic',
    author_email='ivanpauno@ekumenlabs.com',
    maintainer='Aditya Pande, Brandon Ong, William Woodall',
    maintainer_email='aditya.pande@openrobotics.org, brandon@openrobotics.org, william@openrobotics.org',  # noqa: E501
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
