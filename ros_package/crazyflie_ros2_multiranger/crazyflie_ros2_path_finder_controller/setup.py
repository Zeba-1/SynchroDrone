from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_path_finder_controller'
submodule_name = 'crazyflie_ros2_path_finder_controller/algorithms'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodule_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastien SAUTIER',
    maintainer_email='sebsautier18@gmail.com',
    description='Path finder controller for the Crazyflie',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_finder_controller = crazyflie_ros2_path_finder_controller.path_finder_controller:main',
        ],
    },
)
