from setuptools import setup

package_name = 'pubsub'

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
    maintainer='Jeonghyun Kim, Seojung Eom, Yechan Jung',
    maintainer_email='jhkim21@hanyang.ac.kr',
    description='Aruco marker and BLE beacon based positioning system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rss = pubsub.nRSS:main',
            'aruco = pubsub.nAruco:main',
            'position = pubsub.nPosition:main',
        ],
    },
)
