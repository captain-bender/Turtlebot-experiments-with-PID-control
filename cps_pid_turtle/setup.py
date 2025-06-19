from setuptools import find_packages, setup

package_name = 'cps_pid_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/cps_pid_turtle/launch', ['launch/follow_shape.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bender',
    maintainer_email='a.plastropoulos.229@cranfield.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "p_controller = cps_pid_turtle.p_controller:main",
            "sensor_sim   = cps_pid_turtle.sensor_sim:main",
        ],
    },
)
