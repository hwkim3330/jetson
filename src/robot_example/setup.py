from setuptools import setup

package_name = 'robot_example'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KETI',
    maintainer_email='keti@keti.re.kr',
    description='Example nodes for KETI Robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_detection = robot_example.obstacle_detection:main',
            'patrol = robot_example.patrol:main',
            'move_forward = robot_example.move_forward:main',
            'turn = robot_example.turn:main',
        ],
    },
)
