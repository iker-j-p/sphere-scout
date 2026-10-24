from glob import glob

from setuptools import find_packages, setup

package_name = 'sphere_scout_ai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO(kickoff)',
    maintainer_email='todo@todo.todo',
    description='Sphere-Scout thermal detection and survivor localization.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'thermal_celsius_node = sphere_scout_ai.thermal_celsius_node:main',
            'print_thermal = sphere_scout_ai.print_thermal:main',
            'thermal_detector_node = sphere_scout_ai.thermal_detector_node:main',
            'target_localizer_node = sphere_scout_ai.target_localizer_node:main',
        ],
    },
)
