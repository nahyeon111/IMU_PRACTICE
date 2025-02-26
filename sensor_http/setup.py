from setuptools import setup
import os
from glob import glob

package_name = 'sensor_http'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # sensor_http_launch.py만 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/sensor_http_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_http_server = sensor_http.sensor_http_server:main',
            'imu_processor = sensor_http.imu_processor:main',
            'imu_kalman_filter = sensor_http.imu_kalman_filter:main',  
        ],
    },
)

