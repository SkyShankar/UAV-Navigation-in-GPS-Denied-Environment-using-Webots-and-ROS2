import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mavic2_ekf_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akash_shankar',
    maintainer_email='akash_shankar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = mavic2_ekf_pkg.ekf_node:main',
            'ekf_node_follower_scen1 = mavic2_ekf_pkg.ekf_node_follower_scen1:main',
            'ekf_node_follower_scen2 = mavic2_ekf_pkg.ekf_node_follower_scen2:main',
            'sdre_node = mavic2_ekf_pkg.sdre_node:main',
            'logger_node_ekf = mavic2_ekf_pkg.logger_node_ekf:main',
            'logger_node_follower_scen1 = mavic2_ekf_pkg.logger_node_follower_scen1:main',
            'logger_node_follower_scen2 = mavic2_ekf_pkg.logger_node_follower_scen2:main',
            'key_board_controller = mavic2_ekf_pkg.key_board_controller:main',
            'follower_control_node = mavic2_ekf_pkg.follower_control_node:main',
            'logger_node_leader_scen1 = mavic2_ekf_pkg.logger_node_leader_scen1:main',
            'logger_node_leader_scen2 = mavic2_ekf_pkg.logger_node_leader_scen2:main',
            'logger_node_sdre = mavic2_ekf_pkg.logger_node_sdre:main'
        ],
    },
)
