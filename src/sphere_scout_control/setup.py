from setuptools import find_packages, setup

package_name = 'sphere_scout_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO(kickoff)',
    maintainer_email='todo@todo.todo',
    description='Sphere-Scout motor state machine (drive, stop, expand, scan, retract).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_state_machine = sphere_scout_control.motor_state_machine:main',
        ],
    },
)
