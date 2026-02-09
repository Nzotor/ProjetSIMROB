from glob import glob
from setuptools import setup

package_name = 'tello_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/cfg', glob('cfg/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clyde McQueen',
    maintainer_email='clyde@mcqueen.net',
    description='Tello ROS driver (Python simplified for simulation)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_joy = tello_driver.tello_joy:main',
            'tello_keyboard = tello_driver.tello_keyboard:main',
        ],
    },
)
