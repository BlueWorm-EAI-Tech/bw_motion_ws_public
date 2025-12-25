from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bw_sim2real_view'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='menlin',
    maintainer_email='menlinyan@gmail.com',
    description='Unified UI panel and view launch for bw_sim2real.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mantis_control_panel = bw_sim2real_view.mantis_control_panel:main',
        ],
    },
)
