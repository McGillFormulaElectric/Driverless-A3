from setuptools import setup
from glob import glob

package_name = 'a3_neil'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Neil George',
    maintainer_email='neilgeorge03@gmail.com',
    description='MFE A3 skidpad TF scenario, grader, and rosbag2 input recorder.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_scenario_node = a3_neil.tf_scenario_node:main',
            'grader = a3_neil.grader:main',
            'record_inputs = a3_neil.record_inputs:main',
        ],
    },
)
