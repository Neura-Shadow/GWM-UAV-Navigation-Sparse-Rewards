from setuptools import find_packages, setup

setup(name="gwm_px4_control", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/gwm_px4_control"]),
                  ("share/gwm_px4_control", ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"],
      entry_points={"console_scripts": ["p2_control = gwm_px4_control.node:main"]})
