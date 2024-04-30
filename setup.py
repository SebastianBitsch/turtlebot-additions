from setuptools import find_packages, setup

package_name = 'turtle_localization'

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
    maintainer='sebastianbitsch',
    maintainer_email='sebastianbitsch@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = turtle_localization.calculate_odometry:main',
            'mapping = turtle_localization.publish_map:main',
            'frontier = turtle_localization.frontier_publisher:main'
        ],
    },
)
