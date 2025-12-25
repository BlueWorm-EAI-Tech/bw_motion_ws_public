from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bw_sim2real'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='menlin',
    maintainer_email='menlinyan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_to_real_bridge = bw_sim2real.sim_to_real_bridge:main',
            'input_router = bw_sim2real.input_router:main',
            'mantis_casadi_node = bw_sim2real.mantis_casadi_node:main',
            'mantis_playback_node = bw_sim2real.mantis_playback_node:main',
            'vel_config = bw_sim2real.vel_config:main',
        ],
    },
)
