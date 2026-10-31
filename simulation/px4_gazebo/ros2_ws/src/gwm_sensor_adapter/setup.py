from setuptools import find_packages, setup

setup(name="gwm_sensor_adapter", version="0.3.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/gwm_sensor_adapter"]),
                  ("share/gwm_sensor_adapter", ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"],
      entry_points={"console_scripts": ["p3_depth_adapter = gwm_sensor_adapter.node:main"]})
