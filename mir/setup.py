from setuptools import find_packages, setup

package_name = 'mir'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lass6230',
    maintainer_email='due6230@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mir_server_node = mir.mir_server_node:main',
            'mir_position_node = mir.mir_check_position_node:main',
        ],
    },
)
