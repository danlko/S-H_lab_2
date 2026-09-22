from setuptools import setup

package_name = 'fpv_lab2'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='Student',
      maintainer_email='student@example.com', description='ArduPilot SITL flight tasks 3 and 4.',
      license='Apache-2.0', entry_points={'console_scripts': [
          'flight_test = fpv_lab2.flight_test:main',
          'custom_flight = fpv_lab2.custom_flight:main']})
