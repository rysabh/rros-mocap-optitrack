from setuptools import find_packages, setup

package_name = 'mocap_service'

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
    maintainer='battery',
    maintainer_email='rishabhshukla2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_service_subscriber = mocap_service.mocap_service_subscriber:main',
            'mocap_service_client = mocap_service.mocap_service_client:main',
        ],
    },
)
