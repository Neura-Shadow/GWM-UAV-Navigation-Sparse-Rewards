from setuptools import find_packages, setup

setup(name="gwm_navigation_recovery", version="0.8.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/gwm_navigation_recovery"]),
                  ("share/gwm_navigation_recovery", ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"],
      entry_points={"console_scripts": ["p8_p4r_control = gwm_navigation_recovery.control_node:main_p4r",
                                        "p8_p5r_control = gwm_navigation_recovery.control_node:main_p5r"]})
