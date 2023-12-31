#!/usr/bin/env python3.7

import rospy
# import cv_bridge
from std_msgs.msg import String
from std_msgs.msg import Int32

import cv2
import pyrealsense2 as rs
import mediapipe as mp
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import copy


class SkeletonTree:
    def __init__(self, pos):
        self.children = []
        self.pos = pos

class Skeleton:

    def __init__(self):
        
        # INITIALIZATION REGARDING MEDIAPIPE
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence = 0.5, min_tracking_confidence = 0.5)

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
        

    # Get the pose (skeleton coordinates) of the person in the image
    def get_skeleton(self, color_image):

        pose_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        results = self.pose.process(pose_image)

        skel_image = cv2.cvtColor(pose_image, cv2.COLOR_RGB2BGR)

        self.mp_drawing.draw_landmarks(skel_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS, landmark_drawing_spec = self.mp_drawing_styles.get_default_pose_landmarks_style())

        return results, skel_image
    
    # Returns the mean point of given 2 points
    def mean_point(self, point_a, point_b):

        return (point_a + point_b)/2

    def landmark_to_3D(self, joint):

        point = np.array([joint.x, -joint.y, joint.z])

        visibility = joint.visibility

        return point, visibility
    
    def upper_pose_from_landmark(self, landmark):

        point_LEFT_HIP, visibility_LEFT_HIP = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.LEFT_HIP])
        point_RIGHT_HIP, visibility_RIGHT_HIP = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.RIGHT_HIP])

        point_LEFT_SHOULDER, visibility_LEFT_SHOULDER = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER])
        point_RIGHT_SHOULDER, visibility_RIGHT_SHOULDER = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER])

        point_NOSE, visibility_NOSE = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.NOSE])

        point_LEFT_ELBOW, visibility_LEFT_ELBOW = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.LEFT_ELBOW])
        point_RIGHT_ELBOW, visibility_RIGHT_ELBOW = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.RIGHT_ELBOW])

        point_LEFT_INDEX, visibility_LEFT_INDEX = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.LEFT_INDEX])
        point_RIGHT_INDEX, visibility_RIGHT_INDEX  = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.RIGHT_INDEX])

        point_LEFT_PINKY, visibility_LEFT_PINKY  = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.LEFT_PINKY])
        point_RIGHT_PINKY, visibility_RIGHT_PINKY  = self.landmark_to_3D(landmark[self.mp_pose.PoseLandmark.RIGHT_PINKY])

        # TORSO
        point_TORSO = self.mean_point(self.mean_point(point_LEFT_HIP,point_RIGHT_HIP), self.mean_point(point_LEFT_SHOULDER,point_RIGHT_SHOULDER))
        visibility_TORSO = self.mean_point(self.mean_point(visibility_LEFT_HIP,visibility_RIGHT_HIP), self.mean_point(visibility_LEFT_SHOULDER,visibility_RIGHT_SHOULDER))
        # NECK
        point_NECK = self.mean_point(point_LEFT_SHOULDER,point_RIGHT_SHOULDER)
        visibility_NECK = self.mean_point(visibility_LEFT_SHOULDER,visibility_RIGHT_SHOULDER)
        # HEAD
        point_HEAD = point_NOSE  
        visibility_HEAD = visibility_NOSE  

        # LEFT HAND
        point_LEFT_HAND = self.mean_point(point_LEFT_INDEX,point_LEFT_PINKY)
        visibility_LEFT_HAND = self.mean_point(visibility_LEFT_INDEX,visibility_LEFT_PINKY)
        # RIGHT HAND
        point_RIGHT_HAND = self.mean_point(point_RIGHT_INDEX,point_RIGHT_PINKY)
        visibility_RIGHT_HAND = self.mean_point(visibility_RIGHT_INDEX,visibility_RIGHT_PINKY)

        # Changing the skeleton configuration (to correspond the one used by Butepage)
        pose_upper = np.array(point_TORSO)
        pose_upper = np.vstack([pose_upper,point_NECK])
        pose_upper = np.vstack([pose_upper,point_HEAD])
        pose_upper = np.vstack([pose_upper,point_LEFT_SHOULDER])
        pose_upper = np.vstack([pose_upper,point_RIGHT_SHOULDER])
        pose_upper = np.vstack([pose_upper,point_LEFT_ELBOW])
        pose_upper = np.vstack([pose_upper,point_RIGHT_ELBOW])
        pose_upper = np.vstack([pose_upper,point_LEFT_HAND])
        pose_upper = np.vstack([pose_upper,point_RIGHT_HAND])

        visibility = np.array(visibility_TORSO)
        visibility = np.vstack([visibility, visibility_NECK])
        visibility = np.vstack([visibility, visibility_HEAD])
        visibility = np.vstack([visibility, visibility_LEFT_SHOULDER])
        visibility = np.vstack([visibility, visibility_RIGHT_SHOULDER])
        visibility = np.vstack([visibility, visibility_LEFT_ELBOW])
        visibility = np.vstack([visibility, visibility_RIGHT_ELBOW])
        visibility = np.vstack([visibility, visibility_LEFT_HAND])
        visibility = np.vstack([visibility, visibility_RIGHT_HAND])

        return pose_upper, visibility

    def translate_joint(self, node_root: SkeletonTree, joints, translation):

        parent = node_root.pos
        joints[parent] = joints[parent] - translation
        
        for child in node_root.children:
            joints = self.translate_joint(child, joints, translation)
        
        return joints

    def scale_pose_up(self, node_root: SkeletonTree, joints, scaling_factor):

        queue = []
        queue.append(node_root)

        while(len(queue) > 0):

            node = queue[0]
            queue.pop(0)

            parent = node.pos

            vec_parent = joints[parent]

            for child in node.children:
            
                vec_child = joints[child.pos]
                norm = scaling_factor 
                # That is the position that we want this joint to be
                vec_final = vec_parent + norm*(vec_child - vec_parent)  
                joints = self.translate_joint(child, joints, vec_child - vec_final)
                queue.append(child)

        return joints 
    
    def center_pose_torso(self, node_root: SkeletonTree, joints):

        translation = copy.deepcopy(joints[node_root.pos])
        joints_cetered = self.translate_joint(node_root, joints, translation) 

        return joints_cetered

    def check_pose_diff(self, pose1, pose2):

        if pose1.size == 0 or pose2.size == 0:
            print("Some of the poses could not be obtained")
            return -1

        pose1 = pose1.reshape((1,-1))
        pose2 = pose2.reshape((1,-1))

        diff = pose1 - pose2
        square = diff**2
        # square = square.reshape((N_TIMEWINDOWS, 3*N_JOINTS))

        mse = square.sum()

        # Instead of 10, I should define a variable with this limit
        if mse > 10:
            # Then we have an error
            print("This pose is very different from the previous one. There is some error")
            return -1

        return 0    

    def plot_skeleton_Full_Window(self, node_root, joints, limb_length):

        fig = plt.figure(figsize=(6,6))
        ax = fig.add_subplot(projection = '3d')

        # joints = self.scale_pose_up(node_root, joints, limb_length)

        queue = []
        queue.append(node_root)

        while(len(queue) > 0):

            node = queue[0]

            print(joints[node.pos])

            queue.pop(0)

            for child in node.children:
                queue.append(child)
                x = np.append(joints[node.pos][0] , joints[child.pos][0])
                y = np.append(joints[node.pos][1] , joints[child.pos][1])
                z = np.append(joints[node.pos][2] , joints[child.pos][2])

                norm = (x[0] - x[1])**2 + (y[0] - y[1])**2 + (z[0] - z[1])**2
                norm = math.sqrt(norm)

                ax.plot(x, y, z, marker = 'o')

        plt.show()

class estimate_pose:

    def __init__(self) -> None:

        # INITIALIZATION REGARDING ROS PUBLISHER
        self.pub_pose = rospy.Publisher('pose_human', String, queue_size=50)

        self.pub_stop_robot = rospy.Publisher('stop_robot', Int32, queue_size=50)
        self.period_pub_stop = 30

        # INITIALIZATION REGARDING REALSENSE

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.image_width = 640
        self.image_height = 480

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        # There values are needed to calculate the mapping
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)
        self.depth_min = 0.11 #meter
        self.depth_max = 10.0 #meter

        # WE DON'T ALIGN ALL THE POINTS IN THIS CASE (ONLY THE FEW THAT ARE NECESSARY). THEREFORE, WE COMMENT THE LINES BELOW
        # # Create an align object
        # # rs.align allows us to perform alignment of depth frames to others frames
        # # The "align_to" is the stream type to which we plan to align depth frames.
        # self.align_to = rs.stream.color
        # self.align = rs.align(self.align_to)


        self.depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.color_intrin = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.depth_to_color_extrin =  self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.color))
        self.color_to_depth_extrin =  self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( self.profile.get_stream(rs.stream.depth))

        # INITIALIZATION REGARDING MEDIAPIPE
        self.skel = Skeleton()

        # Initialize counter
        self.counter = 0

        # Define check factor and other variables regarding occlusion of joints
        self.CHECK_FACTOR_LENGTH = 0.20
        self.occlusion_shoulder = 0

        # Define the check MSE (to see if two consecutive poses changed much or not)
        self.test_MSE = 0


    def loop(self):

        # Increase the counter
        self.counter = self.counter + 1

        if self.counter % 1800 == 0:
            print("\n\n PASSED 1 MINUTE \n\n ")

        self.color_frame, self.depth_frame, self.color_image, self.skel_depth_image = self.get_camera_frame()

        if self.counter % 1 == 0:
            self.result_joints, self.skel_image = self.skel.get_skeleton(self.color_image)

            # Check if someone was detected
            if self.result_joints.pose_world_landmarks == None:
                print("NOBODY DETECTED IN THE IMAGE")
                return -1

            self.u_pose, self.visibility = self.skel.upper_pose_from_landmark(self.result_joints.pose_world_landmarks.landmark)

            self.occlusion_shoulder = 0
            scale_factor = self.get_scale_factor(self.visibility, self.result_joints, self.skel, better_joints=0)
            if scale_factor == 0:
                print("Scale factor was 0, and that cannot happen")
                return -2

            # Scale the pose up based on the scale_factor obtained
            self.pose_torso_ref_aux = self.skel.scale_pose_up(self.skel.torso, copy.deepcopy(self.u_pose), scale_factor)

            # Center the pose -> the coordinates of the torso go to (0,0,0)
            self.pose_torso_ref = self.skel.center_pose_torso(self.skel.torso, copy.deepcopy(self.pose_torso_ref_aux))

            self.torso_real_coord = self.get_torso_real_coord(self.visibility, self.result_joints, self.skel, self.pose_torso_ref, self.occlusion_shoulder)
            if self.torso_real_coord.size == 0:
                print("Couldnt get the torso real coordinate")
                return -3

            self.pose_lab_ref = self.get_pose_lab_ref(self.torso_real_coord, self.pose_torso_ref, self.skel)
            if self.torso_real_coord.size == 0:
                print("Couldnt get the pose in the lab reference frame (global ref)")
                return -4

            # Check if our previous pose is similar or not (if it is not, then we have an error)
            if self.test_MSE == 1:
                check = self.skel.check_pose_diff(self.pose_lab_ref, self.last_pose_lab_ref)
                
                if check == -1:
                    print("This pose is very different from the previous one. There is some error")
                    return -5

            self.publish_pose(self.pose_torso_ref, self.torso_real_coord)

            if self.counter % self.period_pub_stop == 0:
                #Publish here saying that the robot should not stop
                stop_robot = 0
                self.pub_stop_robot.publish(stop_robot)

            self.last_pose_lab_ref = self.pose_lab_ref
            self.test_MSE = 1

            # cv2.imshow("Depth with skeleton", self.skel_depth_image)
            cv2.imshow("Color with skeleton", self.skel_image)
            # cv2.imshow("Color with skeleton", self.skel_image)
            cv2.waitKey(1) 

            # Everything went well (A person was in fact detected)
            return 0   

    # Get the latest frames from RealSense Camera
    def get_camera_frame(self):

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return
    
        # Convert images to numpy arrays
        # Doing that we can use cv2 without trouble (cv2 and numpy are compatible)
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        return color_frame, depth_frame, color_image, depth_colormap

    # TODO Maybe change some hard coded values for parameters (400 for example)
    def check_camera_covered(self, depth_frame, dist_close = 0.4):
        # Check somehow if the the camera is covered or not
        # If the camera is covered, we send a msg to stop the robot. Stop robot becomes 1
        stop = False
        stop_robot = 0
        n_close_points = 0
        for y in range(100,380, 10):
            for x in range(100,540, 10):
                dist = depth_frame.get_distance(x, y)
                if dist < dist_close:
                    n_close_points += 1
                    # If more than 400 points are close, then we consider that the robot is covered
                    if n_close_points > 400:
                        stop = True
                        stop_robot = 1
                        break
            if stop == True:
                break

        self.pub_stop_robot.publish(stop_robot)

    def publish_pose(self, pose_torso_ref, torso_real_coord):

        msg_pose = self.create_msg_pose(pose_torso_ref, torso_real_coord)
        # This if is kind of useless right now
        if msg_pose == None:
            return -1

        self.pub_pose.publish(msg_pose)
        return 0

    def create_msg_pose(self, pose, torso_coord):
        
        # This if is kind of useless right now
        if torso_coord.size == 0 or pose.size == 0:
            print("We dont have a torso coord or pose to send")
            return 

        msg = ""
        for i in range(9):
            for j in range(3):

                value = pose[i][j]
                msg = msg + "{:.4f}".format(value) + ","

        for j in range(3):
     
            value = torso_coord[j]
            msg = msg + "{:.4f}".format(value) + ","       

        return msg

    def get_pose_lab_ref(self, torso_real_coord, pose_torso_ref, skel: Skeleton):

        pose_lab_ref = skel.translate_joint(skel.torso, copy.deepcopy(pose_torso_ref), -1*torso_real_coord)
        return pose_lab_ref

    # Get the coordinates of the torso considering that the RealSense gives the right values
    def get_torso_real_coord(self, visibility, result_joints, skel: Skeleton, pose_torso_ref, occlusion_shoulder = 0):
        # So, after having the pose in the TORSO reference (Torso, point equals (0,0,0))

        # Lets calculate torso position based on left shoulder position
        if visibility[skel.left_shoulder.pos] >= 0.8 and occlusion_shoulder == 0: 
            
            pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_SHOULDER]
            real_coord = self.get_3Dpoint_from_pixel(pixel_coord)

            if real_coord == None:
                print("Couldnt get the torso real coord")
                return np.array([])

            torso_coord = real_coord - pose_torso_ref[skel.left_shoulder.pos]

            return torso_coord

        # Lets calculate torso position based on right shoulder position
        elif visibility[skel.right_shoulder.pos] >= 0.8 and occlusion_shoulder == 0:
            
            pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            real_coord = self.get_3Dpoint_from_pixel(pixel_coord)

            if real_coord == None:
                print("Couldnt get the torso real coord")
                return np.array([])

            torso_coord = real_coord - pose_torso_ref[skel.right_shoulder.pos]
            return torso_coord

        # Lets calculate torso position based on head position
        elif visibility[skel.head.pos] >= 0.8:
            
            pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.NOSE]
            real_coord = self.get_3Dpoint_from_pixel(pixel_coord)

            if real_coord == None:
                print("Couldnt get the torso real coord")
                return np.array([])

            torso_coord = real_coord - pose_torso_ref[skel.head.pos]
            return torso_coord
        
        else:
            print("Could not get the TORSO coordinates in the lab frame")
            return np.array([])


    # Get the coordinates of a point in the real world (x,y,z) given the coordinates of the pixel of that point 
    def get_3Dpoint_from_pixel(self, pixel_coordinates):

        try:
            # The pixel_coordinates that we receive varies from 0 to 1 (it is normalized). So, we need to multiply by the length (pixels) in x and y
            pixel_coord = [int(pixel_coordinates.x * self.image_width), int(pixel_coordinates.y * self.image_height)]
            
            depth_pixel = rs.rs2_project_color_pixel_to_depth_pixel(
                    self.depth_frame.get_data(), self.depth_scale,
                    self.depth_min, self.depth_max,
                    self.depth_intrin, self.color_intrin, self.depth_to_color_extrin, self.color_to_depth_extrin, pixel_coord)
            
            x_depth_pixel, y_depth_pixel = depth_pixel

            x_depth_pixel = int(x_depth_pixel)
            y_depth_pixel = int(y_depth_pixel)

            if x_depth_pixel == -1 or y_depth_pixel == -1:
                print("The depth_pixel couldnt be calculated/finded")
                return

            depth = self.depth_frame.get_distance(x_depth_pixel, y_depth_pixel)

            if depth == float(0):
                print("The depth calculated was 0")
                return 

            depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [x_depth_pixel, y_depth_pixel], depth)

            x_depth_point, y_depth_point, z_depth_point = depth_point

            oriented_depth_point = [x_depth_point, -y_depth_point, z_depth_point]

            return oriented_depth_point
        
        except:
            print("An exception occured when getting the 3D coordinates of a point")
            return
    
    def distance_2_points(self, point1, point2):

        dist_sqr = (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 + (point1[2] - point2[2])**2
        dist = math.sqrt(dist_sqr)
        
        return dist 
        
    def distance_check(self, point1, point2):

        dist_sqr = (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2
        dist = math.sqrt(dist_sqr)
        
        return dist 

    # Se what is the scale factor between the reality and the estimated by MediaPipe
    def get_scale_factor(self, visibility, result_joints, skel, better_joints):
        
        # We will do that using the distance between the shoulders
        # We will calculate the real distance using realsense functions
        # Then, we will calculate the distance obtained by the mediapipe

        # We cannot trust in the visibility factor (mainly for the shoulders). So, I will check later the distance (if it is wrong, we calculate again)
        # The  variable better_joints will help us choose the right joints

        # If better_joints == 0: we can get any of the combination of joints
        # If better_joints == 1: we should get any of the combination of joints that does not include the shoulders (1st option)
        # If better_joints == 2: we should get any of the combination of joints that does not include 1st option and 2nd option
        # And etc....


        if visibility[skel.left_shoulder.pos] >= 0.9 and visibility[skel.right_shoulder.pos] >= 0.9 and better_joints < 1:
            
            # Coordinates of the pixel where the joint is
            point1_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_SHOULDER]
            point2_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_SHOULDER]

            # Coordinates of the joint in the real world (estimation of the MediaPipe)
            point1_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_SHOULDER]
            point2_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_SHOULDER]    

        
        elif visibility[skel.left_shoulder.pos] >= 0.8 and visibility[skel.left_elbow.pos] >= 0.8 and better_joints < 2:
        
            # Coordinates of the pixel where the joint is
            point1_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_SHOULDER]
            point2_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_ELBOW]

            # Coordinates of the joint in the real world (estimation of the MediaPipe)
            point1_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_SHOULDER]
            point2_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_ELBOW]
        
        elif visibility[skel.right_shoulder.pos] >= 0.8 and visibility[skel.right_elbow.pos] >= 0.8 and better_joints < 3:
            
            # Coordinates of the pixel where the joint is
            point1_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            point2_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_ELBOW]

            # Coordinates of the joint in the real world (estimation of the MediaPipe)
            point1_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            point2_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_ELBOW]

        elif visibility[skel.left_elbow.pos] >= 0.8 and visibility[skel.left_hand.pos] >= 0.8 and better_joints < 4:
            
            # Coordinates of the pixel where the joint is
            point1_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_ELBOW]
            point2_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_WRIST]

            # Coordinates of the joint in the real world (estimation of the MediaPipe)
            point1_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_ELBOW]
            point2_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.LEFT_WRIST]

        elif visibility[skel.right_elbow.pos] >= 0.8 and visibility[skel.right_hand.pos] >= 0.8 and better_joints < 5:
            
            # Coordinates of the pixel where the joint is
            point1_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_ELBOW]
            point2_pixel_coord = result_joints.pose_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_WRIST]

            # Coordinates of the joint in the real world (estimation of the MediaPipe)
            point1_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_ELBOW]
            point2_world_coord = result_joints.pose_world_landmarks.landmark[skel.mp_pose.PoseLandmark.RIGHT_WRIST]
        else:
            print("There are not enough visible points to calculate the scale factor")
            return 0
        
        point1_real_coord = self.get_3Dpoint_from_pixel(point1_pixel_coord)
        point2_real_coord = self.get_3Dpoint_from_pixel(point2_pixel_coord)

        # If the depth is 0 or the point cannot be finded, then try other pair of joints
        if point1_real_coord == None or point2_real_coord == None:
            scale_factor = self.get_scale_factor(visibility, result_joints, skel, better_joints+1)
            return scale_factor

        dist_check = self.distance_check(point1_real_coord, point2_real_coord)

        # Check if this pair of joints is occluded or not (Another check besides visibility)
        if dist_check < self.CHECK_FACTOR_LENGTH:
            self.occlusion_shoulder = 1
            scale_factor = self.get_scale_factor(visibility, result_joints, skel, better_joints+1)
            return scale_factor

        dist_real = self.distance_2_points(point1_real_coord, point2_real_coord)


        point1_estimated_coord = [point1_world_coord.x, point1_world_coord.y, point1_world_coord.z]
        point2_estimated_coord = [point2_world_coord.x, point2_world_coord.y, point2_world_coord.z]

        dist_estimated = self.distance_2_points(point1_estimated_coord, point2_estimated_coord)

        scale_factor = dist_real/dist_estimated

        # Check if scale_factor has an acceptable value:
        if (scale_factor > 0.5) and (scale_factor < 2):
            return scale_factor
        else:
            # We had some kind of error when calculating it... So, we throw an error
            print("The value obtained for the scale factor is not plausible. Therefore, something wrong happened")
            return 0



if __name__ == '__main__':

    rospy.init_node("get_pose_test", anonymous=True)
    node = estimate_pose()

    try:
        while not rospy.is_shutdown():

            # Lets do the following... If the image is bad (nobody on it or with those other factors with wrong results)
            # Then we return a negative number. However, if the image is good, then we return 0
            error = node.loop()

            if error < 0:
                node.test_MSE = 0
                if node.counter % node.period_pub_stop == 0:
                    node.check_camera_covered(node.depth_frame)


    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()