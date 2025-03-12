from setuptools import find_packages, setup

package_name = 'pc_mrg_flt'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FabianJohnTHL',
    maintainer_email='fabian.john@th-luebeck.de',
    description='This project merges two PointCloud2 messages and apply a filter to downscale the resulting PointCloud2. Developed and tested with Ubuntu 22.04 and ROS2 (humble).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
                'pc_mrg_flt = pc_mrg_flt.merge_filter:main'
        ],
    },
)
