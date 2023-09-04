# ROS_HMP_Collision_Avoidance

This repo contains all the code (file and directories) of the ROS packages to run the work related with Human Motion Prediction (HMP) and to run some tests regarding the collision avoidance with VFI.

---------
The hmp_pkg is the package related to the HMP - as the name suggests. Inside it you have two important nodes: get_pose_refactor.py and HMP_trans_continuous.py. 

The first code is the one responsible for initializing the Intel Realsense camera, getting the footage and passing it through the MediaPipe Landmark Detector, so we can get information about the pose of the person in real-time.

The second code is the one that stores the human pose and passes it through the VAE network, so we can get the predictions of approximately 1.5s.

The communication between these two codes/nodes is done using simple ROS topics.

-> The HMP_with_translation.py code is an outdated version of HMP_trans_continuous.py and is probably already deleted from the repo.

-> The .model files are different VAE models built using slightly different configurations (the code to create such models were developed using the colab). In the end, for a continuous prediction (with also the variance to produce a gaussian distribution) we use the model 'vt_continuous.model' and for a discrete prediction (just the mean) we use the model '_vt_d_all_changes.model'. For the translation model we use the 'vt_d_trans.model'


------------
The controller_pkg was the ros package used in the beginning of the project to test the collision avoidance implementation. The tests were performed with the help of the CoppeliaSim simulator. However, in the end of the project the code was being tested and adjusted in the real robot. Hence, the final code related to the collision avoidance with VFI is not in this repo, but in another one called franka_ros. 

-> This pkg has some Coppelia scenes that might be useful for you. If you use the franka_kinematic_human2.ttt and run the ros node test_controller_vfi.cpp, you will see the person's body in the CoppeliaSim in real-time.

