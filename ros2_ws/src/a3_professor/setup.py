from setuptools import setup
from glob import glob

package_name = 'a3_professor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Professor',
    maintainer_email='professor@example.com',
    description='MFE A3 skidpad TF scenario and grader.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_scenario_node = a3_professor.tf_scenario_node:main',
            'grader = a3_professor.grader:main',
        ],
    },
)
