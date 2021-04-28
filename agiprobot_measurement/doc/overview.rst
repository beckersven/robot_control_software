Overview
========

This package provides a pipeline to plan, execute and handle a measurement-trajectory for a given CAD-object.

.. figure:: images/overview.png

The important module is :ref:`trajectory-manager`. It performs the motion planning pipeline with assistance by using the other modules :ref:`viewpoint` and :ref:`sensor-model`. 

During planning, :ref:`trajectory-manager` samples the surface of the target-mesh into discrete surface points and generates straight measurement-trajectories based on those points.
Starting at each surface point, the program places the *anchor* of a possible trajectory above this sample point and then processes the corresponding straight trajectory metrologically and mechanically.
To do so, the :ref:`sensor-model` is used to determine which other sample-points would be visible during this trajectory and how their uncertainties are. `MoveIt <https://moveit.ros.org/>`_ is used to
review, if and how the trajectory-candidate is actually executable in the modeled scene (collision-awareness, reachablility, ...). Both information - the sensor-evaluation and the kinematics - are then stored
into a :ref:`viewpoint`-object. An algorithm is used to determine from the set of all viewpoints one subset, that covers all sampled surface points with an adjustable objective (like "minimize amount of viewpoints"), i.e. solves
the Set Covering Problem.
Afterwards, the chosen :ref:`viewpoint`-objects are connected in a time-optimal way and the combined trajectory by connecting all viewpoint-trajectories and their in-between-segments can be executed or stored for later execution.



   