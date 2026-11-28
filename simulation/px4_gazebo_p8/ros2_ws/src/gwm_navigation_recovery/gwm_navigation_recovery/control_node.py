"""P8 controller entry points: `p8_p4r_control` and `p8_p5r_control` (contract `p8-navigation-recovery-v1` §4, §5).

The nodes are `p8_p4r_node` and `p8_p5r_node`, generated from the accepted P4 and P5 nodes by
`simulation/px4_gazebo_p8/scripts/p8_nodes.py` and checked by a test. Safe to import without ROS.
"""


def main_p4r(argv=None):
    from .p8_p4r_node import main
    return main(argv)


def main_p5r(argv=None):
    from .p8_p5r_node import main
    return main(argv)
