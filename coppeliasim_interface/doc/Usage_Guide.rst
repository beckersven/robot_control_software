Usage Guide
===========
.. role:: bash(code)

Start-Up
--------

To launch the CoppeliaSim-Interface, one has to do following steps in **correct order**. In the beginning, no ROS or CoppeliaSim should be running. 
**Be sure to always source required bash-files!**

#. Open 3 terminal windows
#. In the first terminal, start the ROS-framework with :bash:`roscore`
#. In the second one, navigate via :bash:`cd` to CoppeliaSim and launch it: :bash:`./coppeliaSim.sh`
#. Wait until CoppeliaSim is launched, then open and configure the :bash:`basic_scene.ttt` (see the separate documentation)
#. In the third terminal, run: :bash:`roslaunch coppeliasim_interface simulation_bringup.launch`
#. Wait until MoveIt's GUI is fully loaded and perform basic planning-/execution-tasks to ensure functionality

Scanner
-------
* Output
    * :bash:`/current_scan_line` (:bash:`sensor_msgs/PointCloud`): What the Laser-Scanner measures at the moment
    * :bash:`/stitched_point_cloud` (:bash:`sensor_msgs/PointCloud`): Combined point cloud of all measurements while stitching was active
    * Visualization in RVIZ
        #. At the bottom of the left panel "Displays", click "Add"
        #. Switch the right tab and add the above topic(s)
        #. Optional: In the left panel "Displays", customize the point cloud's appearance (you might need to scroll down to do so)
    * Color matches sensor precision: Green = low uncertainty, red = high uncertainty, gradient inbetween
* Stitch-Control
    * Service :bash:`/clear_stitched_point_cloud` (:bash:`std_srvs/Trigger`): Clear the stitched point cloud
    * Service :bash:`/switch_stitch_mode` (:bash:`std_srvs/SetBool`): Enables stitching, if request's :bash:`data` is set :bash:`true`, or disables it otherwise
    * Stitching is disabled on start-up