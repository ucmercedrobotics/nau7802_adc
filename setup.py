from setuptools import find_packages, setup

package_name = 'nau7802_adc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labrnth',
    maintainer_email='lbooth@ucmerrced.edu',
    description='A ROS2 wrapper for the Cedargrove/Adafruit NAU7802 ADC library (and sensor)',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['nau7802_node = nau7802_adc.nau7802_adc:main'
        ],
    },
)