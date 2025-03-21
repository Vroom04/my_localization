from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'state_estimation_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), 
            glob('launch/*.launch.py')),
        (os.path.join('share/', package_name), 
            glob('robot_localization/*.yaml')),
         (os.path.join('share', package_name), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='95103311+ScudeT@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_rg=state_estimation_py.remove_imu_gravity:main',
            'height_est=state_estimation_py.height_estimation:main',
            'imu_merger_node=state_estimation_py.imu_merger_node:main'
        ],
    },
)
