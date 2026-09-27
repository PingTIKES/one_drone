from setuptools import setup

package_name = 'one_drone_navigation'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/config', ['config/params.yaml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='PingTIKES',
      maintainer_email='team@example.com', description='Single-drone map alignment and Nav2 goal interface',
      license='MIT', entry_points={'console_scripts': [
          'map_odom = one_drone_navigation.map_odom:main',
          'height_slice = one_drone_navigation.height_slice:main',
          'goal_manager = one_drone_navigation.goal_manager:main']})
