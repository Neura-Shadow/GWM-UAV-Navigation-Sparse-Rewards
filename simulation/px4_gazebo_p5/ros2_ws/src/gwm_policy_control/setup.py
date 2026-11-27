from setuptools import find_packages, setup

setup(name="gwm_policy_control", version="0.5.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/gwm_policy_control"]),
                  ("share/gwm_policy_control", ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"],
      entry_points={"console_scripts": ["p5_control = gwm_policy_control.control_node:main"]})
