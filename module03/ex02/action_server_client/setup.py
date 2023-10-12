from setuptools import find_packages, setup

package_name = 'action_server_client'

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
    maintainer='nekr',
    maintainer_email='a.nekrasova@g.nsu.ru',
    description='turtle move',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = action_server_client.server:main',
            'client = action_server_client.client:main'
        ],
    },
)
