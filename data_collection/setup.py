from setuptools import find_packages, setup

package_name = 'data_collection'

auto_detected_packages = find_packages(
    exclude=['test', 'test.*']  
)

manually_specified_packages = [
    package_name,
    'data_collection.submodules'
]

all_packages = auto_detected_packages + manually_specified_packages


setup(
    name=package_name,
    version='0.0.0',
    packages=all_packages,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rishabh Shukla',
    maintainer_email='shuklar@usc.edu',
    description='Data collection package for the project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'camera_recorder = data_collection.camera_recorder:main',
            # 'experiment_recorder = data_collection.experiment_recorder:main',
            # 'wrench_recorder = data_collection.wrench_recorder:main',
            # 'robot_interface = data_collection.robot_interface:main',
            # 'experiment_executor = data_collection.experiment_executor:main',
            # 'PIH_state_machine = data_collection.PIH_state_machine:main',
            # 'impedence_recorder = data_collection.impedence_recorder:main',
            # 'repeatability_test = data_collection.repeatability_test:main',
            # 'ati_wrench_writer = data_collection.ati_wrench_writer:main',
            # 'ati_wrench_publisher = data_collection.ati_wrench_publisher:main',
            'multi_sensor_recorder = data_collection.multi_sensor_recorder:main',
            'export_take = data_collection.export_take:main',
            'arduino_blue = data_collection.arduino_blue:main',
            'arduino_green = data_collection.arduino_green:main',
            'switchboard_router = data_collection.switchboard_router:main',
            'screen_recorder = data_collection.screen_recorder:main',
            'force_plot_video = data_collection.force_plot_video:main',
        ],
    },
)
