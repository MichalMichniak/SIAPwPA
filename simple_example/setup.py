from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'simple_example'

def generate_data_files(share_path, dir):
    data_files = []
    
    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*')))
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='evs@agh.edu.pl',
    description='Simple example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_gazebo_ex = simple_example.simple_example_node:main'
        ],
    },
)
