.. _viewpoint:

View Point
==========

Introduction and Basic Ideas
----------------------------
make latexpdf
A ViewPoint is a container of one measurement-trajectory with extra-functionality. 

It can store besides the actual measurement-trajectory another trajectory towards its 
measurement trajectory as well as quantitative metrological predictions about the measurement. A View can construct itself in a bare geometrical sense: Given a surface point,
a anchor-position as well as an orientation of the laser_emitter_frame are created so that the z-axis of this frame points towards this surface_point, when in anchor-position.

Commonly Used Terms
:::::::::::::::::::
* surface-point: The sampled surface point on the target-mesh this viewpoint is based on
* anchor-position: The position this viewpoint's trajectory goes through and where its laser_emitter_frame's z-axis points towards the corresponding surface-point
* laser_emitter_frame: Frame positioned at the the tip of the scanners's fan that is moved along the straight trajectory during execution
    * x-axis points in the trajectory-direction
    * z-axis points towards the corresponding surface-point when it becomes measured
    * y-axis results from the right-handedness of the coordinate-system
    * During trajectory-execution, the orientation of this frame is constant in the 'world'-frame but the position changes
* angle-around-boresight: Angle for rotation of this viewpoints straight-trajectory around the laser_emitter_frame's z-axis in anchor-position

Visualization of Concepts
:::::::::::::::::::::::::

.. figure:: images/view_geometry.png

   


API-Specification
-----------------
.. automodule:: agiprobot_measurement.viewpoint
   :members:
   :show-inheritance: