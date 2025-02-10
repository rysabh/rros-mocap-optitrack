from setuptools import find_packages, setup

package_name = 'kuka_blue_pose_pub'

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
            'kuka_blue_pose_pub = kuka_blue_pose_pub.kuka_pose_pub:main',
        ],
    },
)
