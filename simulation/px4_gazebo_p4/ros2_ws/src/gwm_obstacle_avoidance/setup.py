from setuptools import find_packages, setup

setup(name="gwm_obstacle_avoidance", version="0.4.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/gwm_obstacle_avoidance"]),
                  ("share/gwm_obstacle_avoidance", ["package.xml"])],
      install_requires=["setuptools"], tests_require=["pytest"],
      entry_points={"console_scripts": ["p4_control = gwm_obstacle_avoidance.control_node:main",
                                        "p4_perception = gwm_obstacle_avoidance.perception_node:main"]})
