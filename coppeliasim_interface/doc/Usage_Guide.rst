Usage Guide
===========
.. role:: bash(code)

To use the CoppeliaSim-Interface, one has to do following steps in **correct order**. In the beginning, no ROS or CoppeliaSim should be running. 
**Be sure to always source required bash-files!**

#. Open 3 terminal windows
#. In the first one, start the ROS-framework with :bash:`roscore`
#. In the second, set the sim-time-parameter true through: :bash:`rosparam set /use_sim_time true`
#. Also in this terminal, navigate via :bash:`cd` to CoppeliaSim and launch it: :bash:`./coppeliaSim.sh`
#. Wait until CoppeliaSim is launched, then open and configure the :bash:`basic_scene.ttt`
#. In the third terminal, run: :bash:`roslaunch coppeliasim_interface simulation_bringup.launch`
#. Wait until MoveIt's GUI is fully loaded and perform basic planning-/execution-tasks to ensure functionality
