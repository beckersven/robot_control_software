.. _view-planning-detailed:

Path Planning Detailed
======================

0. Load the Task
----------------

Read in the given CAD-file of the target-object as well as its pose. The mesh is made available to the local trimesh-handler and loaded into the motion-planning-framework 
`MoveIt <https://moveit.ros.org>`_ so that it will be considered for collision-checks.

.. figure:: images/moveit_collision_added.png

   

1. Preprocessing and Sampling
-----------------------------

Removes all downward facing surface faces since they are not considered measurable. This is based on the common assumption that the fixture holds the target object from below.
Use `trimesh <https://trimsh.org/>`_ to sample the mesh's surface into discrete surface-points depending on a specified sampling-density. The samples, corresponding face-normals and the target-mesh are
set as context in a :ref:`sensor-model`-instance.

.. figure:: images/sampling_trimesh.png

2. ViewPoint-Generation and -Evaluation
---------------------------------------

For each-sample, generate :ref:`viewpoint`-objects by moving into the points face-normal and then applying rotations to get a variety of candidates per sampled surface point.
The resulting point is the anchor of the viewpoint. Each viewpoint is assigned a straight trajectory-line. 

.. figure:: images/trajectory_generation.png


After that, 2 main evaluations are executed:

Metrological Evaluation
:::::::::::::::::::::::

* Use the :ref:`sensor-model`-instance to analyze, which other sample-points are visible from this trajectory, and at what uncertainty
* Result = Visibilities and uncertainties (will be stored in the :ref:`viewpoint`-object)

.. figure:: images/measurement_example.png
   
  Higher sampling density than previously for vividness. Red points are immeasurable; Green points are measurable at low uncertainty, blue ones at high uncertainty.

Mechanical Evaluation
:::::::::::::::::::::

* Utilize `MoveIt <https://moveit.ros.org>`_ to examine, if this trajectory is actually executable in the scene (collision, reacability, ...).
* **Critical**: If the mechanical evaluation fails, the viewpoint will not be considered any further
* Can also consider trajectory-parts: If e.g. 80% of the trajectory are be executable, this :ref:`viewpoint`-object might not be rejected
* Result = List of actual joint-values (will be stored in the :ref:`viewpoint`-object)


.. figure:: images/collision_check.png

The sequence of these evaluations is implemented in a way to minimize compute-time.

3. ViewPoint-Selection (Set Covering Problem)
---------------------------------------------

When a set of valid :ref:`viewpoint` s has been found, not all :ref:`viewpoint` -elements must be executed for a valid result: A subset must be found that 
covers the same surface-points as all the found :ref:`viewpoint` s (= *Set Covering Problem*). This subset is in general much smaller than the original set and can be built using different algorithms:

As the first option, 'Greedy' implementation (both used in the papers) selects in each iteration that new :ref:`viewpoint` from the original set, that can contribute the most 
not-yet-visible surface-sample-points to the subset. If 2 or more :ref:`viewpoint` -objects can contribute the same amount, the one with the lowest uncertainty will be selected.

Another option is Integer Programming (IP):

Minimize :math:`\sum_{\forall i} c_j v_j` subject to :math:`\sum_{\forall j} m_{i,j} v_j \ge 1~~~~\forall i` 

where :math:`v_j \in \{0,1\}` indicates if viewpoint j is element of the subset and :math:`m_{i,j} \in \{0, 1\}`, if the sampled surface point i is measurable by the trajectory of viewpoint j.
:math:`c_{j}` is a cost-term. If it is constant, the IP-problems is identically to the Greedy approach. However, by assigning :math:`c_{i}` the time of the measurement-trajectory
of viewpoint i or its uncertainty, a bigger variety of solutions becomes possible than in the first option.

4. Connecting the ViewPoints
----------------------------

Lastly, the determined set of :ref:`viewpoint`-objects must be ordered for optimal execution. To do so, the current robot-pose is enqueued in an 'execution-list'.
Paths from the endpoint of the last element of execution-list are calculated to every non-enqueued viewpoint's trajectory-start- and -endpoint. The :ref:`viewpoint` with the shortest path-time becomes enqueued.
Also, the inter-viewpoint path will be stored in that :ref:`viewpoint` so that it will perform the exact same path during execution (if the path would get planned again dynamically,
it might be completely different due to the stochastic nature of path-planning). This step relies again on `MoveIt <https://moveit.ros.org>`_.

