"""Linux-side Gen3 VIVE arm teleop helpers.

This package is intentionally separate from the Wuji Hand ``get_fingers_data``
input-device path.  The Windows VIVE sender is externally managed; modules here
only validate and consume its UDP/ROS2 contract on Linux.
"""
