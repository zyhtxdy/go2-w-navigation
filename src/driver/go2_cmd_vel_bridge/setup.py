from setuptools import find_packages, setup

package_name = 'go2_cmd_vel_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/go2_cmd_vel.yaml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='zyh',
    maintainer_email='zyh@todo.todo',
    description='Bridge planar Twist commands to the Unitree Go2 Sport API.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['cmd_vel_bridge = go2_cmd_vel_bridge.node:main']},
)
