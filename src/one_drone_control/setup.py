from setuptools import setup

package_name = 'one_drone_control'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/config', ['config/params.yaml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='PingTIKES',
      maintainer_email='team@example.com', description='Nav2 velocity to PX4 1.14.3 Offboard bridge',
      license='MIT', entry_points={'console_scripts': [
          'flight_bridge = one_drone_control.flight_bridge:main']})
