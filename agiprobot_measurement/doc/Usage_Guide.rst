Usage Guide
===========
.. role:: bash(code)

To use the trajectory planner, just start the trajectory_manager_user.py in a terminal where the workspace was sourced. You can do this by executing :bash:`rosrun agiprobot_measurement trajectory_manager.py`. 
The following procedure is quite self-explanatory if a basic understanding of the corresponding bachelor thesis is present. However, consider following notices:

* The STL-file you want to plan for must be in the "workpieces"-folder (there already exist three default parts that were used as "Testobjekte" in the thesis)
* Full trajectories can be loaded or stored in the "trajectories"-folder as yaml-files
* The identifier in parentheses () corresponds to the variable's name in the thesis
* The value in brackets [] is the default value - If there is no input and enter is pressed, this value is used
    * If this is a yes/no-question, the capital letter in the brackets corresponds to default (e.g. ....?[y/N] and enter-press with no input is equivalent to a "N" input)
* When loading a STL- or trajectory-file, autocompletion vie <TAB> is supported!
* The algorithm can be stopped at any time via :bash:`CTRL`+:bash:`C`
* If there is a stop-request during execution, the current trajectory-segment will still be fully executed (for safety reasons)
    * When working with the real robot, use the red e-stop button to stop the robot immediateley in case of an emergeny!
