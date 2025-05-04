from setuptools import setup

package_name = 'my_tracking_package'

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
    maintainer='yoon',
    maintainer_email='yoon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_tracking_node = my_tracking_package.object_tracking_node:main',
            'turtlebot_move_node = my_tracking_package.turtlebot_move_node:main',
            'marker_tracking_turtlebot4_node = my_tracking_package.marker_tracking_turtlebot4_node:main',
            'turtlebot_controller_node = my_tracking_package.turtlebot_controller_node:main'
        ],
    },
)
