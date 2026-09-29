from setuptools import setup

package_name = 'ego_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=['src'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='PingTIKES', maintainer_email='team@example.com',
    description='OpenVINS odometry and depth-health adapter for EGO-Planner',
    license='MIT',
    entry_points={'console_scripts': ['ego_odom_adapter = src.ego_odom_adapter:main']},
)
