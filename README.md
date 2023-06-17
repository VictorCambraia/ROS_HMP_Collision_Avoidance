# ROS_HMP_Collision_Avoidance
This repo wil contain all the code (files and directories) of the ROS workspace to run the entire work related with HMP and collision avoidance (controller + VFI part)

-----------------------------------
TODOs of the repo

I have just run for the first time in coppeliaSIm the system completely integrated (HMP + VFI + controller).
And I noticed some things that I need to think how I solve them

1 - If the camera doesnt capture the human motion, then we consider that the robot is free to move or not?
  The first idea that came through my mind was: If the camera is being blocked (that means if there is  something blocking the view), then the robot should stop. However, if the  camera is "watching" everything, then we should probably just keep going (there is nobody present). (DONE, I think)
  
2 - What should I do when the solver cannot find a solution? I should probably just stop the robot right? (DONE, I think)

3 - I NEED to take in account that the robot base is not in the (0,0,0) postion (DONE, I think)

4 - I NEED to take in account the variance of the points!!! (I NEED TO TEST)

5 - I NEED to change the Model VAE (the sample z). Right now it only considers z = z_mu, which is wrong.

6 - I NEED TO TEST everything: the variance, the 3 jacobians and the stop of the robot.

TODOs that are not a must (but would be good if I could implement them)

-> Search for others datasets (I actually dont think I will have enough time for this)

--------------------------------
Where were the codes before this repo?

So, before creating this repo and this new catkin/ros workspace.  The codes were splitted in different directories in the PC from the uni and also in my personal notebook. So, it was kind of a mess. Anyway, I will try to register here where were everything, so if I need to recover something someday, I remeber about it.

In my personal PC:
-> HMP code was in the catkin_ws_HMP (path: /Area de Trabalho/Tese/catkin_ws_HMP/src/mediapipe_pkg)
I actually thought about expanding this workspace instead of creating a new one. But I think it will be easier and cleaner just to create another one, and get just the good part (this was not a git directoy, which was bad)

-> VFI Controller code was in the GIT_controller_code (path: /Area de Trabalho/dqrobotics/GIT_controller_code/VFI_Controller/cpp_main_codes)
This directory was "connected" with git, however it was not a ROS workspace, and now I need to run these codes in ROS...

In the uni PC:
-> HMP code was in the catkin_test (path: victor/catkin_test)
The name of the directory is pretty bad (created a long time ago as test), and is not a git directory. 

-> VFI Controller code was in the GIT_controller_code (path: victor/DQrobotics/GIT_Controler_code/VFI_Controller)
This directory was "connected" with git, however it was not a ROS workspace, and now I need to run these codes in ROS...

So, as we can see it is better to just create a new ROS workspace connected with git and that has everythinng....
