#!/usr/bin/env python3.7

import tensorflow as tf
# from tensorflow import keras
import rospy
import cv_bridge 
import cv2
from std_msgs.msg import String
import numpy as np
import math
import os
import copy

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def decode_msg(msg):

    N_JOINTS = 9

    values = msg.split(',')
    values = list(map(float, values[:-1]))

    values_np = np.array(values)

    points = np.reshape(values_np,(N_JOINTS+1,3))

    torso_coord = points[-1,:]
    pose = points[0:N_JOINTS,:]

    return pose, torso_coord

def calculate_MSE(vec1, vec2):

    N_JOINTS = 8
    N_TIMEWINDOWS = 50

    diff = vec1 - vec2
    square = diff**2

    square = square.reshape((N_TIMEWINDOWS, 3*N_JOINTS))

    mse = square.sum(axis=1)
    return mse

class SkeletonTree:
    def __init__(self, pos):
        self.children = []
        self.pos = pos

class Skeleton:

    def __init__(self):

        # Lets create the skeleton tree now:
        self.torso = SkeletonTree(0)
        self.neck = SkeletonTree(1)
        self.head = SkeletonTree(2)
        self.left_shoulder = SkeletonTree(3)
        self.right_shoulder = SkeletonTree(4)
        self.left_elbow = SkeletonTree(5)
        self.right_elbow = SkeletonTree(6)
        self.left_hand = SkeletonTree(7)
        self.right_hand = SkeletonTree(8)

        self.torso.children.append(self.neck)
        self.neck.children.append(self.head)
        self.neck.children.append(self.left_shoulder)
        self.neck.children.append(self.right_shoulder)
        self.left_shoulder.children.append(self.left_elbow)
        self.right_shoulder.children.append(self.right_elbow)
        self.left_elbow.children.append(self.left_hand)
        self.right_elbow.children.append(self.right_hand)
        

    def translate_joint(self, node_root: SkeletonTree, joints, translation):

        parent = node_root.pos
        joints[parent] = joints[parent] - translation
        
        for child in node_root.children:
            joints = self.translate_joint(child, joints, translation)
        
        return joints

    def scale_pose_up(self, node_root: SkeletonTree, joints, limb_length):

        N_JOINTS = 9

        # The zeros were added previously, so now we dont need to add it one more time
        # joints = np.append([0,0,0],joints)
        joints = np.reshape(joints, (N_JOINTS,3))

        queue = []
        queue.append(node_root)

        while(len(queue) > 0):

            node = queue[0]
            queue.pop(0)

            parent = node.pos
            vec_parent = joints[parent]

            for child in node.children:
            
                vec_child = joints[child.pos]
                # norm = limb_length 
                norm = limb_length[child.pos - 1]
                # That is the position that we want this joint to be
                vec_final = vec_parent + norm*(vec_child - vec_parent)  
                joints = self.translate_joint(child, joints, vec_child - vec_final)
                queue.append(child)

        return joints 
    
    def scale_pose_down(self, node_root: SkeletonTree, joints, limb_length):

        queue = []
        queue.append(node_root)

        while(len(queue) > 0):

            node = queue[0]
            queue.pop(0)

            parent = node.pos
            vec_parent = joints[parent]

            for child in node.children:
            
                vec_child = joints[child.pos]

                norm = 1/limb_length[child.pos - 1]
                # That is the position that we want this joint to be
                vec_final = vec_parent + norm*(vec_child - vec_parent)  
                joints = self.translate_joint(child, joints, vec_child - vec_final)
                queue.append(child)

                size_limb = self.distance_2_points(joints[parent], joints[child.pos])

        return joints 
    
    def distance_2_points(self, point1, point2):

        dist_sqr = (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2
        dist = math.sqrt(dist_sqr)
        
        return dist 


    def get_size_limbs(self, joints):

        N_JOINTS = 9
        limb_length = np.zeros(N_JOINTS-1)

        queue = []
        queue.append(self.torso)

        while(len(queue) > 0):

            node = queue[0]
            queue.pop(0)

            parent = node.pos
            vec_parent = joints[parent]

            for child in node.children:
            
                vec_child = joints[child.pos]
                limb_length[child.pos - 1] = self.distance_2_points(vec_parent, vec_child)

                queue.append(child)

        return limb_length
    
    def plot_skeleton_Full_Window(self, node_root, joints, limb_length):

        N_JOINTS = 9
        joints = np.reshape(joints, (N_JOINTS,3))

        fig = plt.figure(figsize=(6,6))
        # ax = fig.add_subplot(projection = '3d')
        ax = fig.add_subplot()

        # joints = self.scale_pose_up(node_root, joints, limb_length)

        queue = []
        queue.append(node_root)

        while(len(queue) > 0):

            node = queue[0]

            queue.pop(0)

            for child in node.children:
                queue.append(child)
                x = np.append(joints[node.pos][0] , joints[child.pos][0])
                y = np.append(joints[node.pos][1] , joints[child.pos][1])
                z = np.append(joints[node.pos][2] , joints[child.pos][2])

                norm = (x[0] - x[1])**2 + (y[0] - y[1])**2 + (z[0] - z[1])**2
                norm = math.sqrt(norm)

                # ax.plot(x, y, z, marker = 'o')
                ax.plot(x, y, marker = 'o')

        plt.show()

    def plot_skeleton_Full_Window2(self, node_root, joints1, joints2):

        N_JOINTS = 9
        joints1 = np.reshape(joints1, (N_JOINTS,3))
        joints2 = np.reshape(joints2, (N_JOINTS,3))

        fig = plt.figure(figsize=(6,6))
        # ax = fig.add_subplot(projection = '3d')
        ax = fig.add_subplot()

        # joints = self.scale_pose_up(node_root, joints, limb_length)

        for i in range(2):
            if i == 0:
                joints = joints1
            else:
                joints = joints2

            queue = []
            queue.append(node_root)  

            while(len(queue) > 0):

                node = queue[0]
                queue.pop(0)

                for child in node.children:
                    queue.append(child)
                    x = np.append(joints[node.pos][0] , joints[child.pos][0])
                    y = np.append(joints[node.pos][1] , joints[child.pos][1])
                    z = np.append(joints[node.pos][2] , joints[child.pos][2])

                    # ax.plot(x, y, z, marker = 'o')
                    ax.plot(x, y, marker = 'o')

        plt.show()


class HMP:

    def __init__(self, continuous_model) -> None:

        self.counter = 0
        self.timer_prev = rospy.get_time()
        self.time_sec = 0
        self.time_min = 0

        # INITIALIZATION REGARDING ROS PUBLISHER
        self.pub = rospy.Publisher('prediction_human', String, queue_size=50)

        # self.CHECK_PERIOD = 10
        self.CHECK_PERIOD = 0.5

        # The parameter that decides if the model is continuous or not
        self.continuous_model = continuous_model

        # Create a clear pose matrix
        # Just make it clear: there are actually 2 pose matrix: one is normalized and the other not
        self.reset_pose_matrix()

        # Create a clear prediction_matrix()
        self.reset_prediction_matrix()

        # Initialize the skeleton, so we can do the scale_pose_down()
        self.skel = Skeleton()

        py_file= os.path.abspath(__file__) # path to main.py
        py_dir = os.path.dirname(py_file) # path to the parent dir of main.py
        # model_path = os.path.join(py_dir, '_vt_d.model') 

        if continuous_model:
            model_path = os.path.join(py_dir, 'vt_continuous.model') 
        else:
            model_path = os.path.join(py_dir, '_vt_d_all_changes.model')

        model_path_torso = os.path.join(py_dir, 'vt_d_trans.model') 

        self.model = tf.keras.models.load_model(model_path)
        self.model_torso = tf.keras.models.load_model(model_path_torso)

        self.model.summary()
        self.model_torso.summary()

        # INITIALIZATION REGARDING ROS SUBSCRIBER
        rospy.Subscriber("pose_human", String, self.cb_motion_prediction)

    
    def publish_pose(self, prediction, log_sigma):

        # Make sure that the prediction is just a vector (and not a matrix)
        prediction = prediction.reshape(-1)
        log_sigma = log_sigma.reshape(-1)
        msg_pose = self.create_msg_pose(prediction, log_sigma)
        if msg_pose == None:
            return -1

        self.pub.publish(msg_pose)
        return 0

    def create_msg_pose(self, prediction, log_sigma):

        # I actually dont think this can happen
        if prediction.size == 0:
            print("We dont have a prediction to send")
            return 

        msg = ""

        for i in range(prediction.shape[0]):
            msg = msg + "{:.4f}".format(prediction[i]) + ","

        for i in range(log_sigma.shape[0]):
            msg = msg + "{:.4f}".format(log_sigma[i]) + ","
    
        return msg
        

    def reset_pose_matrix(self):
        self.pose_matrix = np.array([])
        self.pose_matrix_normalized = np.array([])

        # Adding the TORSO part
        self.torso_matrix = np.array([])

    def reset_prediction_matrix(self):
        self.prediction_matrix = np.array([])
        self.prediction_scaled_matrix = np.array([])
        self.log_sigma_complete_matrix = np.array([])
    
    def add_pose_matrix(self, pose_skel, torso_coord):

        # From 50 to 50 iterations we update the limb_length
        if self.pose_matrix.size == 0 or self.counter % 50 == 0:
            self.scale_factor_array = self.skel.get_size_limbs(pose_skel)

        # Scale Down the pose
        pose_skel_normalized = self.skel.scale_pose_down(self.skel.torso, pose_skel, self.scale_factor_array)

        # The first row should have only zeros. So, we will remove it
        pose_skel = pose_skel[1:, :]
        pose_skel_normalized = pose_skel_normalized[1:, :]

        # To facilitate, now pose_skel has only one row
        pose_skel = np.reshape(pose_skel, (1,-1))
        pose_skel_normalized = np.reshape(pose_skel_normalized, (1,-1))

        # Just to make sure that it will have just one row
        torso_coord = np.reshape(torso_coord, (1,-1)) 

        if self.pose_matrix.size == 0:

            self.pose_matrix = np.array(pose_skel)
            self.pose_matrix_normalized = np.array(pose_skel_normalized)
            self.torso_matrix = np.array(torso_coord)

            return 0
        
        elif self.pose_matrix.shape[0] < 50:

            self.pose_matrix = np.vstack([self.pose_matrix, pose_skel])
            self.pose_matrix_normalized = np.vstack([self.pose_matrix_normalized, pose_skel_normalized])
            self.torso_matrix = np.vstack([self.torso_matrix, torso_coord])

            return 0
        
        elif self.pose_matrix.shape[0] == 50:
            
            # x[:-1] = x[1:]; x[-1] = newvalue
            self.pose_matrix[:-1,:] = self.pose_matrix[1:,:]
            self.pose_matrix[-1] = pose_skel

            self.pose_matrix_normalized[:-1,:] = self.pose_matrix_normalized[1:,:]
            self.pose_matrix_normalized[-1] = pose_skel_normalized

            self.torso_matrix[:-1,:] = self.torso_matrix[1:,:]
            self.torso_matrix[-1] = torso_coord

            return 1
        
    def add_prediction_matrix(self, new_prediction, new_prediction_scaled, new_log_sigma_complete):

        # Now we have 2 prediction matrices, one with the normalized data, and other complete (scaled + torso)

        # Guarantee that new_prediction has only one row
        new_prediction = np.reshape(new_prediction, (1,-1))
        new_prediction_scaled = np.reshape(new_prediction_scaled, (1,-1))
        new_log_sigma_complete = np.reshape(new_log_sigma_complete, (1,-1))

        if self.prediction_matrix.size == 0:

            self.prediction_matrix = np.array(new_prediction)
            self.prediction_scaled_matrix = np.array(new_prediction_scaled)
            self.log_sigma_complete_matrix = np.array(new_log_sigma_complete)
            return 0
        
        elif self.prediction_matrix.shape[0] < 50+1:

            self.prediction_matrix = np.vstack([self.prediction_matrix, new_prediction])
            self.prediction_scaled_matrix = np.vstack([self.prediction_scaled_matrix, new_prediction_scaled])
            self.log_sigma_complete_matrix = np.vstack([self.log_sigma_complete_matrix, new_log_sigma_complete])

            return 0
        
        elif self.prediction_matrix.shape[0] == 50+1:
            
            # x[:-1] = x[1:]; x[-1] = newvalue
            self.prediction_matrix[:-1,:] = self.prediction_matrix[1:,:]
            self.prediction_matrix[-1] = new_prediction

            self.prediction_scaled_matrix[:-1,:] = self.prediction_scaled_matrix[1:,:]
            self.prediction_scaled_matrix[-1] = new_prediction_scaled

            self.log_sigma_complete_matrix[:-1,:] = self.log_sigma_complete_matrix[1:,:]
            self.log_sigma_complete_matrix[-1] = new_log_sigma_complete

            return 1
        
    def evaluate_prediction(self):

        N_JOINTS = 8
        N_TIMEWINDOWS = 50
        
        # Guarantee that new_prediction has only one row
        past_poses_normalized = np.reshape(self.pose_matrix_normalized, (1,-1))

        predicted_poses = self.prediction_matrix[0]

        diff = predicted_poses - past_poses_normalized
        square = diff**2
        square = square.reshape((N_TIMEWINDOWS, 3*N_JOINTS))

        # Motion Prediction Error (MSE)
        mpe = square.sum(axis=1)

        return mpe
            
    # So, I was thinking right now, and I decided to return 2 predictions: 1- the normalized one 2- The complete one (with torso and everything)
    def predict_motion(self):

        input_predict = self.pose_matrix_normalized.reshape(-1,1200)
        input_torso_predict = self.torso_matrix.reshape(-1,150) # Also for the TORSO part

        # log_sigma is the error assoicated with the prediction
        if self.continuous_model:
            (prediction, log_sigma) = self.model([input_predict, input_predict])
            prediction = np.array(prediction[0])
            log_sigma = np.array(log_sigma[0])
        else:
            prediction = self.model([input_predict, input_predict])
            prediction = np.array(prediction[0])
            log_sigma = np.zeros(prediction.shape)

        prediction_torso = self.model_torso([input_torso_predict, input_torso_predict])[0]
        prediction_torso = np.array(prediction_torso)

        # NOW WITH THE OFFSET DOING A KIND OF FADED
        single_pose = np.array(input_predict[0,1176:])
        diff_single = prediction[:24] - single_pose

        single_torso = np.array(input_torso_predict[0,147:])
        diff_single_torso = prediction_torso[:3] - single_torso

        diff_faded = np.array([])
        diff_faded_torso = np.array([])
        alpha = 1
        FADE_RATE = 0.95
        for j in range(50):

            diff_faded = np.append(diff_faded, alpha*diff_single)
            diff_faded_torso = np.append(diff_faded_torso, alpha*diff_single_torso)
            alpha = alpha* FADE_RATE

        diff_faded = diff_faded.reshape(-1,1200)
        diff_faded_torso = diff_faded_torso.reshape(-1,150)

        pred_faded = prediction - diff_faded
        pred_faded_torso = prediction_torso - diff_faded_torso

        prediction = pred_faded
        prediction_torso = pred_faded_torso

        prediction_scaled = copy.deepcopy(prediction.reshape(50,24))
        prediction_scaled = np.hstack([np.zeros((prediction_scaled.shape[0],3)),prediction_scaled])

        # We also need to do the same for the log_sigma (combine torso to the others joints)... The torso error to be considered will be 0. And that is for two reasons
        # 1 - It would be very small, because in fact the torso is very predictable
        # 2 - We would sum the torso error in all the joints, so  it wouldn't make any final difference, when comparing the joints

        # We also don't need to scale the log_sigma because there will be a constant in the future already scaling it up
        log_sigma_complete = log_sigma.reshape(50,24)
        # Since we are dealing with log_sigma, we should put -inf and not 0. So, lets use -99. exp(-99) is basically 0
        log_sigma_complete = np.hstack([-99*np.ones((log_sigma_complete.shape[0],3)),log_sigma_complete])

        prediction_torso = prediction_torso.reshape(50,-1)

        # for i in range(1):
        for j in range(50):
            joints = prediction_scaled[j,:].copy()
            
            # Just to make sure
            joints = joints.reshape(1,27)
            joints = self.skel.scale_pose_up(self.skel.torso, joints, self.scale_factor_array)

            joints_lab_ref = self.skel.translate_joint(self.skel.torso, joints, -1*prediction_torso[j])
            # joint_lab_ref becomes (1,27) instead of (9,3)
            joints_lab_ref = joints_lab_ref.reshape(1,-1)

            prediction_scaled[j,:] = joints_lab_ref

        # prediction_scaled becomes (1,27*50) instead of (50,27)
        prediction_scaled = prediction_scaled.reshape(1,-1)
        log_sigma_complete = log_sigma_complete.reshape(1,-1)

        return prediction, prediction_scaled, log_sigma_complete
    

    def cb_motion_prediction(self, data):

        time_now = rospy.get_time()     

        self.counter += 1

        if (self.counter-1) % 1800 == 0:
            print("\n\n\n\n\n\n \n\n PASSED 1 MINUTE      \n\n\n\n\n\n\n\n")   

        if self.counter % 1 == 0:

            # Convert to real string type, so it is easier to work
            data = str(data)
            # Just to remove the header "data:
            data = data[7:-1]

            # I still need to implement this on the other node (Right now there is no msg with the data "nothing")
            # But the program actually works fine without this 
            if data == 'nothing':
                print("The tracking of the skeleton was lost")
                self.reset_pose_matrix()
                self.reset_prediction_matrix()
                           
            else:
                # Check if the message was received with the right frequency (not very precise, but still helps)
                if (time_now - self.timer_prev) > self.CHECK_PERIOD:

                    print("RESET POSE MATRIX")
                    self.reset_pose_matrix()
                    self.reset_prediction_matrix()
                        
                # The last_pose variable is a matrix with N_JOINTS ROWS AND 3 COLUMNS
                # The torso_real_coors is a normal point [x,y,z] style
                self.last_pose, self.last_torso_real_coord = decode_msg(data)

                # Scale Down is done inside the add_pose_matrix
                run_vae_decision = self.add_pose_matrix(self.last_pose, self.last_torso_real_coord)
                # Run the prediction if we have all the 50 frames
                if run_vae_decision == 1:
                        
                    self.last_prediction, self.last_prediction_scaled, self.last_log_sigma_complete = self.predict_motion()
                    self.publish_pose(self.last_prediction_scaled, self.last_log_sigma_complete)
                    run_evaluate_prediction = self.add_prediction_matrix(self.last_prediction, self.last_prediction_scaled, self.last_log_sigma_complete)

                    # Call the evaluation of the prediction if the matrix is full
                    if run_evaluate_prediction == 1:
                        
                        if self.counter % 90 == 0:

                            # mpe = self.evaluate_prediction()
                            # print("\n\n MPE IS HERE  ",mpe, "\n\n")

                            ########## TEST ZONE  ############

                            # print("\n",self.pose_matrix_normalized,"\n")

                            poses_pred = self.prediction_matrix[50]
                            single_pose = poses_pred[0:24]

                            test = np.array([])

                            for i in range(50):
                                test = np.append(test, single_pose)

                            test = test.reshape(-1,1200)
                            prediction = self.model.predict([test, test])[0]

                            # error_static = calculate_MSE(test, self.pose_matrix_normalized.reshape(1,-1))
                            error_predicted = calculate_MSE(test, poses_pred)

                            # print(" SUPOSE STOPPED PERSON (STATIC ERROR)\n\n",error_static, "\n\n")
                            print(" SUPOSE STOPPED PERSON (ERROR PREDICTED)\n\n",error_predicted, "\n\n")


        time_now2 = rospy.get_time() 

        time_prev = self.timer_prev
        self.timer_prev = time_now2


if __name__ == '__main__':

    rospy.init_node("human_motion_prediction", anonymous=True)
    node = HMP(continuous_model=True)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
