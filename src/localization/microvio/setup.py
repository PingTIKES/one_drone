from setuptools import setup

setup(
    name='microvio', version='0.1.0',
    packages=['microvio'], package_dir={'microvio': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/microvio']),
        ('share/microvio', ['package.xml', 'README.md']),
        ('share/microvio/config', ['config/params.yaml']),
        ('share/microvio/launch', ['launch/microvio.launch.py']),
    ],
    install_requires=['setuptools', 'numpy', 'pymavlink>=2.4.41,<3'], zip_safe=True,
    maintainer='PingTIKES', maintainer_email='team@example.com',
    description='Read PX4 odometry aided by a TELEM-connected MicoVIO module', license='MIT',
    entry_points={'console_scripts': ['microvio = microvio.microvio_node:main']},
)
