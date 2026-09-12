from setuptools import find_packages, setup

setup(name='gwm_sensor_adapter', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/gwm_sensor_adapter']),
                  ('share/gwm_sensor_adapter',['package.xml'])],
      install_requires=['setuptools'], tests_require=['pytest'],
      entry_points={'console_scripts':['p3_sensor = gwm_sensor_adapter.node:main']})
