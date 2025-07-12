from setuptools import find_packages, setup

package_name = 'hardware_data'

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
    maintainer='orin_nano1',
    maintainer_email='basso.gio97@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_data_pub = hardware_data.hardware_data_pub:main',
            'hardware_data_sub = hardware_data.hardware_data_sub:main',
        ],
    },
)
