from setuptools import setup
import glob
package_name = 'adma_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rschilli',
    maintainer_email='rico.schillings@hs-offenburg.de',
    description='tool collection for working with ADMA and ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros2csv = adma_tools.ros2csv_converter:main',
            'bag2gsdb = adma_tools.bag2gsdb_converter:main'
        ],
    },
)
