from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'interface_protocol'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='htx1',
    maintainer_email='your.email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    }
)