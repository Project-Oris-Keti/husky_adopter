from setuptools import setup

package_name = 'husky_adopter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project-Oris-Keti',
    maintainer_email='sjlee88@keti.re.kr',
    description='ROS 2 adapter that bridges Clearpath Husky sensor/odometry data and a MAVLink-based web GCS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'huskydatahandlernode = husky_adopter.huskydatahandlernode:main',
            'gcsdatahandlernode = husky_adopter.gcsdatahandlernode:main',
        ],
    },
)
