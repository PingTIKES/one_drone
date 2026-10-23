from setuptools import setup

setup(
    name='decision_manager', version='0.1.0',
    packages=['decision_manager'], package_dir={'decision_manager': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/decision_manager']),
        ('share/decision_manager', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='PingTIKES', maintainer_email='team@example.com',
    description='OFFBOARD triggered single-run and cyclic waypoint decisions', license='MIT',
    entry_points={'console_scripts': ['decision_manager = decision_manager.decision_node:main']},
)
