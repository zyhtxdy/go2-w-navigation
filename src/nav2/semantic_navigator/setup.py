from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'semantic_navigator'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='bz',
    maintainer_email='user@example.com',
    description=(
        'Semantic target resolver and Nav2 action bridge for the navigation stack.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_navigator = '
            'semantic_navigator.semantic_navigator_node:main',
            'semantic_goal = semantic_navigator.semantic_goal_cli:main',
        ],
    },
)
