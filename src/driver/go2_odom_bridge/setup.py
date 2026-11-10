from setuptools import find_packages, setup


package_name = 'go2_odom_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/sportmodestate_odom.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyh',
    maintainer_email='zyh@todo.todo',
    description='Read Go2 SportModeState and publish body linear velocity for an EKF.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sportmodestate_odom_bridge = go2_odom_bridge.node:main',
        ],
    },
)
