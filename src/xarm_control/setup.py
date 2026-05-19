from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xarm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'xarm_msgs',
        'xarm_planner',
        'xarm_api',
        'rclpy',
        'fpn_msgs'],
    zip_safe=True,
    maintainer='indro',
    maintainer_email='indro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_xarm = xarm_control.run_xarm:main',
            'xarm_moveit = xarm_control.run_xarm_moveit:main',
            'image_saver_node = xarm_control.kiyo_image_save:main'
        ],
    },
)
