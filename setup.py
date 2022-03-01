from setuptools import setup

package_name = 'pet_ros2_lightbeacon_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stefan Kull',
    maintainer_email='stefan.kull@gmail.com',
    description='ROS2 Python package to toggle PWM(RC) controlled light beacons on/off/strobe/blink.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pet_lightbeacon_node=pet_ros2_lightbeacon_pkg.pet_lightbeacon_node:main"
        ],
    },
)
