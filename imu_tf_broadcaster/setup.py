from setuptools import find_packages, setup

package_name = 'imu_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosub',
    maintainer_email='martinxu1234@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'imu_tf_node = imu_tf_broadcaster.imu_tf_node:main',
        ],
        
    },
)
