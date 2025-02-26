from setuptools import setup

package_name = 'imu_plotter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_sensor.launch.py']),  # 📌 여기에 추가!!
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimjeongmin',
    maintainer_email='kimjeongmin@example.com',
    description='IMU Sensor Plotter Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
                'imu_gravity_processor = imu_plotter.imu_gravity_processor:main',  
                'imu_publisher = imu_plotter.imu_publisher:main',
                'imu_orientation_calculator = imu_plotter.imu_orientation_calculator:main',  
                'imu_orientation_calculator2 = imu_plotter.imu_orientation_calculator2:main',  
                'imu_orientation_calculator3 = imu_plotter.imu_orientation_calculator3:main',
     
        ],
    },
)
