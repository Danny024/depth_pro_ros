from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'depth_pro_ros'
submodule_name = 'depth_pro'
source_dir = 'src'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),

        (os.path.join('lib',package_name, submodule_name, source_dir,submodule_name),
         glob(os.path.join(submodule_name, source_dir, submodule_name,'*.py'))),

        (os.path.join('lib',package_name, submodule_name, source_dir,submodule_name,'cli'),
          glob(os.path.join(submodule_name, source_dir, submodule_name,'cli','*.py'))),

        (os.path.join('lib',package_name, submodule_name, source_dir,submodule_name,'eval'),
           glob(os.path.join(submodule_name, source_dir, submodule_name,'eval','*.py'))),

        (os.path.join('lib',package_name, submodule_name, source_dir,submodule_name,'network'),
           glob(os.path.join(submodule_name, source_dir, submodule_name,'network','*.py'))),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models'),glob(os.path.join('model', '*.pth')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Eneh',
    maintainer_email='danieleneh024@gmail.com',
    description='A ROS2 wrapper for Zero Shot Metric Depth Pro by Apple',
    license='MIT-License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_pro_ros = depth_pro_ros.depth_pro_ros:main',
        ],
    },
)
