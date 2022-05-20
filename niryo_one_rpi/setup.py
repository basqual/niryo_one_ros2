from setuptools import setup
from glob import glob
import os
package_name = 'niryo_one_rpi'
setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'config'), glob('config/*'))
    ],    install_requires=['setuptools'],
    zip_safe=True,
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
    'console_scripts': [
        'niryo_one_rpi = niryo_one_rpi.niryo_one_rpi_node:main'
    ],
    },
)