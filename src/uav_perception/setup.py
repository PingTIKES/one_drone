from setuptools import setup

package_name = 'uav_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PingTIKES',
    maintainer_email='team@example.com',
    description='Stereo depth and body-frame obstacle cloud',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stereo_depth_node = uav_perception.stereo_depth_node:main',
            'software_stereo = uav_perception.software_stereo_node:main',
            'sensor_relay = uav_perception.sensor_relay:main',
        ],
    },
)
