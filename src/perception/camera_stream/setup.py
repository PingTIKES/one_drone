from setuptools import setup

package_name = 'camera_stream'
setup(
    name=package_name, version='0.1.0',
    packages=[package_name], package_dir={package_name: 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='PingTIKES', maintainer_email='team@example.com',
    description='camera stream', license='MIT',
    entry_points={'console_scripts': ['sensor_relay = camera_stream.sensor_relay:main']},
)
