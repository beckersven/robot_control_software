Overview
========

.. figure:: images/Coppeliasim_interface_overview.png


**This package is the nexus between ROS and CoppeliaSim.** The :ref:`simulation-synchronizer` ensures that the time in ROS (available through '/clock'-topic) increases with the same rate as in CoppeliaSim.
This is crucial for hard-timed tasks, like simulated real-time-control. This module also calls specified services at each simulation-step to for example read-out the Laser-Scanner or perform one control-cycle.
For the latter, :ref:`hardware-interface` mimics a real ros_control-interface but instead of controlling actual motors, the controller output is forwarded to the joints in CoppeliaSim's UR10e and
the simulated joint-values are used as feedback for the ROS-controllers. This package was designed similarly to the drivers of the real UR-robots, which means that it works
on the one hand seemlessly with every framework developed for real robots (like MoveIt) and on the other hand that movements evaluated in a simulation
can be identically performed in reality when the real driver is loaded instead - *No remapping of ROS-topics, etc. is required*!
The third component, :ref:`point-cloud-manager` , makes the Point-Cloud measured by the scanner in CoppeliaSim accessible to the ROS-system by transforming it. It also performs 
stitching of the measured Point-Cloud as well the application of an uncertainty-model to immediately review the measurement.