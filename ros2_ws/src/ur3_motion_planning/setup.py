from setuptools import setup

package_name = 'ur3_motion_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/ur3_motion_planning.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Domenic Kadioglu',
    maintainer_email='domenic@utsrobo.com',
    description='UR3 Motion Planning and Control for Team Picasso',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planning_node = ur3_motion_planning.motion_planning_node:main',
        ],
    },
)
