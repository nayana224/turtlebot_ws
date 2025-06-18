from setuptools import find_packages, setup

package_name = 'bridge_uart'

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
    maintainer='pyo',
    maintainer_email='inpyoi1304@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_to_uart = bridge_uart.cmdvel_to_uart:main',
            'pose_uart_to_odom = bridge_uart.pose_uart_to_odom:main',
        ],
    },
)
