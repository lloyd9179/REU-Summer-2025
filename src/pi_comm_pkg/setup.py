from setuptools import setup
import os
from glob import glob

package_name = 'pi_comm_pkg'
submodules_dir = 'pi_comm_pkg/submodules/'
submodules_list = [submodules_dir+ 'PCA9685', submodules_dir+ 'TCA9548A', submodules_dir+ 'MCP4725', submodules_dir+ 'MCP3008', submodules_dir+ 'ADS1115', submodules_dir+ 'StepperMotor']

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name] + submodules_list,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools','fpn_msgs'],
    zip_safe=True,
    maintainer='Ben Walt',
    maintainer_email='walt@illinois.edu',
    description='Communications node for the TS robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'pi_comm_pkg_exec = pi_comm_pkg.pi_comm_pkg_exec:main',
            'pan_tilt_pub = pi_comm_pkg.pan_tilt_pub:main',
            'realsense_image_saver_node = pi_comm_pkg.realsense_image_save:main'
            # 'pan_tilt_test = pi_comm_pkg.pan_tilt_tester:main'
        ],
    },
)
