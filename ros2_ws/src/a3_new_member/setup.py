from setuptools import setup
from glob import glob

package_name = 'a3_new_member'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Neil George',
    maintainer_email='neilgeorge03@gmail.com',
    description='MFE A3 student new_member: static + dynamic TF chain.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_node = a3_new_member.static_tf_node:main',
            'dynamic_tf_node = a3_new_member.dynamic_tf_node:main',
        ],
    },
)
