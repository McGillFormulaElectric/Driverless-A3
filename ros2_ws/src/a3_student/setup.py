from setuptools import setup
from glob import glob

package_name = 'a3_student'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='MFE A3 student template.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_node = a3_student.static_tf_node:main',
            'dynamic_tf_node = a3_student.dynamic_tf_node:main',
        ],
    },
)
