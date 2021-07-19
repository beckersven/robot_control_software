.. _hardware-interface:

HardwareInterface C++-class
===========================

This class is heavily inspired by the driver for the UR10e-robot maintained by `Felix Exner at Forschungszentrum Informatik <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>`_. The idea is
to use the hardware abstraction layer (HAL) provided by this class interchangeably with the real HAL sow that from ROS's point of view, there is no
difference. ros_control can use the very same controllers as in reality but instead of sending/receiving signals over ethernet,
this class redirects this information stream towards the simulation - It acts just as a railway switch.

.. doxygenclass:: coppeliasim_interface::HardwareInterface

