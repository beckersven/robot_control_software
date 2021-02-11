.. _trajectory-manager:

Trajectory Manager
==================

Introduction and Basic Ideas
----------------------------

This module provides an configurable system to plan, execute, store and load measurement-trajectories for scanning objects with a laser-line-scanner mounted on a robotic arm.

The planning is heavily inspired by `"View and sensor planning for multi-sensor surface inspection" (Gronle et al., 2016) <https://iopscience.iop.org/article/10.1088/2051-672X/4/2/024009>`_
and `"Model-based view planning" (Scott, 2009) <https://link.springer.com/article/10.1007/s00138-007-0110-2>`_, however this module was adapted to fit this use-case and general improvements were incorporated.

For a given task, the user can specifiy the mesh-to-measure, properties of the trajectory-segments (like lenght) and other parameters.
Then, the planning-pipeline discussed in section :ref:`view-planning-detailed`, will be performed. Afterwards, the generated total trajectory can be executed via
`MoveIt <https://moveit.ros.org>`_ on the real robot or in a simulation (see CoppeliaSim-Interface's documentation in that case). The trajectory can also be 
written to disk for later or repetitive execution. As 'good' trajectories currently require long calculations (>30min), 
this module enables users to load and execute previously computed trajectories.

API-Specification
-----------------

.. automodule:: agiprobot_measurement.trajectory_manager
   :members:
   :show-inheritance:
