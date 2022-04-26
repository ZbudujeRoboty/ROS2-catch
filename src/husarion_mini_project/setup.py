from setuptools import setup

package_name = 'husarion_mini_project'

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
    maintainer='zbuduje_roboty',
    maintainer_email='zbuduje_roboty@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtlebot3_controller = husarion_mini_project.turtlebot3_controller:main",
            "turtlebot3_spawner = husarion_mini_project.turtlebot3_spawner:main"
        ],
    },
)
