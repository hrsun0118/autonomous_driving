#!/usr/bin/env python
# TODO: add feedforward term "ubar" back in the script
# TODO: Wobbling handling is commented out
# TODO: video: https://www.youtube.com/watch?v=Y8WEGGbLWlA
#               https://www.youtube.com/watch?v=cG5c2yfupLI
""" bearing_controller node: subscribe to recceive bearings & turtle pose, and publish cmd_vel."""

import rospy
from me416_environment.msg import Bearings
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
# from gazebo_msgs.msg import ModelStates
from math import cos, acos, pow, sqrt, fmod, pi
import numpy as np

from squaternion import Quaternion

import time

theta_cam = 0
# q_tags_to_world_arr = np.asarray(3 * [Quaternion(1,0,0,0)])
left_tag_id = -1
mid_tag_id = -2
right_tag_id = -3
tag_id = np.asarray([-1, -2, -3])    # create an array of number -1 for 2 times
tags_marked = False         # TODO: tags_marked should NOT be used if: 1) there are more than 2 tags, or 2) if the tags are multi-faced
phi = 0.05
theta_cam_updated = False
last_tag_id = -4
mid_tag_detected = False

def callback_bearings(msg):
    """Callback to receive a message from bearings"""
    global pub_cmd_vel
    global pose
    global theta_cam
    global theta_cam_updated
    global phi

    global tag_id
    global left_tag_id
    global right_tag_id
    global mid_tag_id
    global tags_marked
    global mid_tag_detected

    # wait until theta_cam is obtained before executing this function
    while (theta_cam_updated == False):
        time.sleep(0.002)

    # if 1 or 2 tags detected before marked, exit
    if not tags_marked and msg.len != 3:
        return

    # if 2 tags detected & 1 of them is a mid_tag, exit
    if msg.len == 2 and mid_tag_detected:
        return

    # if 1 tag detected and !mid_tag_detected:
    if msg.len == 1 and not mid_tag_detected:
        return

    # Obtain u
    # 1) set beta0
    beta0_star = np.asarray([[-1], [0]])    # modification: should be  [[0], [-1]] - switch aprial tag frame (x,z) to (z,x) to match w/ Gazebo world frame
    beta1_star = np.asarray([[1], [0]])
    # 2) set 2 arrays to get theta first
    abs_tags = np.zeros(msg.len)
    theta_tags = np.zeros(msg.len)
    # 3.1) calculate "u" (w/ 3 tags)
    if msg.len == 3:
        rospy.loginfo("callback_bearings: 3 tags controller")
        # 3.1.1) obtain left/mid/right tag bearing's angle (theta_left/theta_mid/theta_right)
        abs_tags[0] = sqrt(pow(msg.beta0.x, 2) + pow(msg.beta0.y, 2))
        abs_tags[1] = sqrt(pow(msg.beta1.x, 2) + pow(msg.beta1.y, 2))
        abs_tags[2] = sqrt(pow(msg.beta2.x, 2) + pow(msg.beta2.y, 2))
        theta_tags[0] = acos(msg.beta0.x / abs_tags[0])
        theta_tags[1] = acos(msg.beta1.x / abs_tags[1])
        theta_tags[2] = acos(msg.beta2.x / abs_tags[2])
        rospy.loginfo("theta_tags[0] = {}, theta_tags[1] = {}, theta_tags[2] = {}".format(theta_tags[0], theta_tags[1], theta_tags[2]))

        # 3.1.2) u_bar = np.asarray([[0],[1.75]]) # a feedforward term [1.6 - 1.9] (here = e_dot, which is speed error) (added to help the turtle goes through the gate with speed)
        u = (np.asarray([[msg.beta0.x],[msg.beta0.y]]) \
            + np.asarray([[msg.beta1.x],[msg.beta1.y]]) \
            + np.asarray([[msg.beta2.x],[msg.beta2.y]])) \
            - (beta0_star + beta1_star) # + u_bar # + feedforward term - the direction perpendicular to the gate

        # 3.1.3) find left, mid, right tags' IDs
        if not tags_marked:
            min_id_index = np.argmin(theta_tags)
            max_id_index = np.argmax(theta_tags)
            left_tag_id = tag_id[max_id_index]
            right_tag_id = tag_id[min_id_index]
            for i in range(len(theta_tags)):
                if i != min_id_index and i != max_id_index:
                    mid_tag_id = tag_id[i]

            tags_marked = True
            rospy.loginfo("left_tag_id = {}, mid_tag_id = {}, right_tag_id = {}".format(left_tag_id, mid_tag_id, right_tag_id))

    # 3.2) calculate "u" (w/ 3 tags)
    elif msg.len == 2:
        rospy.loginfo("callback_bearings: 2 tags - left/right tag controller")
        # 3.2.1) 2 tag controller: only drive when "left" & "right" tags are detected
        # obtain left/right tag bearing's angle (theta_left/theta_right)
        abs_tags[0] = sqrt(pow(msg.beta0.x, 2) + pow(msg.beta0.y, 2))
        abs_tags[1] = sqrt(pow(msg.beta1.x, 2) + pow(msg.beta1.y, 2))
        theta_tags[0] = acos(msg.beta0.x / abs_tags[0])
        theta_tags[1] = acos(msg.beta1.x / abs_tags[1])

        # 3.2.2) u_bar = np.asarray([[0],[1.75]]) # a feedforward term [1.6 - 1.9] (here = e_dot, which is speed error) (added to help the turtle goes through the gate with speed)
        u = (np.asarray([[msg.beta0.x],[msg.beta0.y]]) \
            + np.asarray([[msg.beta1.x],[msg.beta1.y]])) \
            - (beta0_star + beta1_star) # + u_bar # + feedforward term - the direction perpendicular to the gate
    elif msg.len == 1:
        rospy.loginfo("callback_bearings: 1 tag - mid_tag controller")
        abs_tags[0] = sqrt(pow(msg.beta0.x, 2) + pow(msg.beta0.y, 2))
        theta_tags[0] = acos(msg.beta0.x / abs_tags[0])

        # 3.2.2) u_bar = np.asarray([[0],[1.75]]) # a feedforward term [1.6 - 1.9] (here = e_dot, which is speed error) (added to help the turtle goes through the gate with speed)
        u = np.asarray([[msg.beta0.x],[msg.beta0.y]]) \
            - (beta0_star + beta1_star) # + u_bar # + feedforward term - the direction perpendicular to the gate

    # set default u & v = 0
    v = 0
    w = 0
    # # deal with turtle wobbling issue; and since v & w == 0, I can avoid deviding cos_theta_u & sin_theta_u by 0
    # min_u = np.asarray([[pow(10, -3)], [pow(10, -3)]])
    # if abs(u[0][0]) < min_u[0][0] and abs(u[1][0]) < min_u[1][0]:
    #     u = np.asarray([[0],[0]])
    #     rospy.loginfo("u =  [%.3f][%.3f], cos=0, sin=1, theta_u = 90 degrees, theta_e= 0", u[0][0], u[1][0]) # [modification] might not need to be hard coded
    # else: # beginning of else indentation
    # calculate theta_u
    abs_u = sqrt(pow(u[0][0], 2) + pow(u[1][0], 2)) # calculate abs_u
    if abs_u != 0:
        cos_theta_u = u[0][0] / abs_u
        # sin_theta_u = u[1][0] / abs_u   # Q: 2 answers for sin_theta_u:
                                        # vector_1 X vector_u = (1,0) X (u1, u2) = u2 - I chose this one - why?
                                        # vector_u X vector_1 = (u1, u2) X (1,0) = -u2
        # if sin_theta_u >= 0:
        theta_u = acos(cos_theta_u)
    else:
        theta_u = 0

    # calculate theta_e
    theta_e = theta_u - theta_cam
    # rospy.loginfo("theta_cam: {:f}, theta_u: {:f}, theta_e: {:f}".format(theta_cam, theta_u, theta_e))
    # rospy.loginfo("u =  [%.3f][%.3f], cos=%.3f, sin=%.3f, theta_u = %.3f, theta_e= %.3f", u[0][0], u[1][0], cos_theta_u, sin_theta_u, theta_u * 180.0 / pi, theta_e)

    # calculate w
    if theta_e < 0:     # theta_u < theta_cam --> robot turn right --> w < 0 from "motor_command_model" implementation of "twist_to_speeds() fn"
        w = - phi
    elif theta_e > 0:   # theta_u > theta_cam --> robot turn left --> w > 0
        w = phi
    # calculate v
    if theta_e != 0:
        v = cos(theta_e) * abs_u / 5
        # rospy.loginfo("cos(theta_cam): {:f}, abs_u:{:f}".format(cos(theta_e), abs_u))
        # rospy.loginfo("v = {:f}".format(v))
    else:
        v = 0
        # rospy.loginfo("v = {:f}".format(v))
    # rospy.loginfo("w = {:f}, v = {:f}".format(w, v))

    # initialize & obtain cmd_vel
    cmd_vel = Twist()
    # linear velocity
    cmd_vel.linear.x = v # [reset back to "v" later] / test 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    # angular velocity
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = w

    # publish w & v
    pub_cmd_vel.publish(cmd_vel) # [uncomment later]

    # reset "theta_cam_updated" & "mid_tag_detected"
    # --> wait for the next "theta_cam" to be updated/ "mid_tag_detected" to be marked detected
    mid_tag_detected = False
    theta_cam_updated = False

    # # log info    # [uncomment later]
    # rospy.loginfo("cmd/vel:")
    # rospy.loginfo("Linear: x = %.3f, y = %.3f, z = %.3f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z)  # uncomment later
    # rospy.loginfo("Angular: x = %.3f, y = %.3f, z = %.3f",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z)  # uncomment later

def callback_tag_detections(tag):
    global pose
    global q_tags_to_world_arr       # Quaternion (april tag0 in the Gazebo World Reference Frame)
    global theta_cam
    global theta_cam_updated

    global tag_id
    global phi
    global pub_cmd_vel
    global tags_marked
    global last_tag_id
    global mid_tag_detected

    global left_tag_id
    global right_tag_id
    global mid_tag_id

    # Get quaternions of camera relative to the apriltag tag0
    theta_arr = np.zeros(len(tag.detections))
    # theta_cam_to_tag_arr = np.zeros(2)
    # rospy.loginfo("new tag:")
    for i in range(len(tag.detections)):
        # get quaternions of camera relative to the apriltagss
        x = tag.detections[i].pose.pose.pose.orientation.x
        y = tag.detections[i].pose.pose.pose.orientation.y
        z = tag.detections[i].pose.pose.pose.orientation.z
        w = tag.detections[i].pose.pose.pose.orientation.w
        q_cam_to_tags = Quaternion(w, x, y, z)                      # get Quaternion of camera to tag
        # rospy.loginfo("tag[{}] = {:f}, {:f}, {:f}, {:f}".format(tag.detections[i].id, w, x, y, z))
        theta_cam_to_tag = q_cam_to_tags.to_euler(degrees=False)    # get euler's angle of camera to tag
        # rospy.loginfo("Euler's angle [{}]: {:f}, {:f}, {:f}".format(tag.detections[i].id[0],theta_cam_to_tag[0], theta_cam_to_tag[1], theta_cam_to_tag[2]))
        theta_arr[i] = theta_cam_to_tag[1] + pi / 2                 # get the 2nd angle (index = 1) - confirmed: it's rotating around the y axis

        # # get quaternions of camera relative to the world: using quaternions multiplication
        # q_tags_to_world = Quaternion(q_tags_to_world_arr[tag.detections[i].id][0], q_tags_to_world_arr[tag.detections[i].id][1], q_tags_to_world_arr[tag.detections[i].id][2], q_tags_to_world_arr[tag.detections[i].id][3])
        # q_cam_to_world = q_cam_to_tags * q_tags_to_world
        # # get Euler's Angles of each tag
        # theta_arr[tag.detections[i].id] = q_cam_to_world.to_euler(degrees=False)[1] # to_euler(): return as a tuple: (roll, pitch, yaw): rotation around (x, y, z) axis, get index = 1 (2nd element)
        #
        # rospy.loginfo("theta(cam to apriltag tag{}): e1: {:.2f}, e2: {:.2f}, e3: {:.2f}".format(tag.detections[i].id, theta_cam_to_tag[0], theta_cam_to_tag[1],  theta_cam_to_tag[2]))

    # 1) Get theta_cam if 2/3 tags are detected
    # 2) Update ID index if 3 tags are detected
    # 3) Publish w to "/robot_twist" if 1 tag is detected
    if len(tag.detections) == 3:
        # update tag id
        if not tags_marked:
            # get tags' ids in a fixed order, & store them in a global array "tag_id"
            index_id = 0;   # for labeling the index of the tags coming in sequence
            for element in tag.detections:
                tag_id[index_id] = element.id[0]
                index_id += 1

        # update theta_cam
        theta_cam = np.sum(theta_arr)/len(tag.detections)
        # rospy.loginfo("theta_cam_arr[0] ={:f}, theta_cam_arr[1] ={:f}".format(theta_arr[0] * 180 / pi, theta_arr[1] * 180 / pi))
        mid_tag_detected = True
        theta_cam_updated = True

    elif len(tag.detections) == 2:
        if tags_marked:
            # if mid_tag detected: mark it to true
            if tag.detections[0].id[0] == mid_tag_id or tag.detections[1].id[0] == mid_tag_id:
                mid_tag_detected = True
            # calculate theta_cam & mark theta_cam_updated to True
            theta_cam = np.sum(theta_arr)/len(tag.detections)
            theta_cam_updated = True
    elif len(tag.detections) == 1:
        if tags_marked:
            rospy.loginfo("tag.detections[0].id[0] = {}, mid_tag_id = {}".format(tag.detections[0].id[0], mid_tag_id))
            if tag.detections[0].id[0] == mid_tag_id:
                # mark mid_tag_detected & last_tag_id
                mid_tag_detected = True
                last_tag_id = mid_tag_id

                rospy.loginfo("Only mid tag is detected")
                # get theta_cam for 1 tag
                theta_cam = np.sum(theta_arr)/len(tag.detections)
                theta_cam_updated = True
                rospy.loginfo("callback_tag_detections: 1 mid tag is detected")
            else:   # take control of the robot: if a single non-mid_tag detected
                # mark the last_tag_id
                if tag.detections[0].id[0] == left_tag_id:
                    last_tag_id = left_tag_id
                elif tag.detections[0].id[0] == right_tag_id:
                    last_tag_id = right_tag_id

                rospy.loginfo("callback_tag_detections: 1 left/right tag is detected")
                # get theta_cam for 1 tag
                theta_cam = np.sum(theta_arr)/len(tag.detections)
                theta_cam_updated = True

                w = 0
                if tag.detections[0].id[0] == left_tag_id:
                    w = -phi
                elif tag.detections[0].id[0] == right_tag_id:
                    w = phi

                # initialize & obtain cmd_vel
                cmd_vel = Twist()
                # linear velocity
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                cmd_vel.linear.z = 0.0
                # angular velocity
                cmd_vel.angular.x = 0.0
                cmd_vel.angular.y = 0.0
                cmd_vel.angular.z = w
                # publish w & v
                pub_cmd_vel.publish(cmd_vel) # [uncomment later]

        # theta_cam = theta_arr[tag.detections[0].id]
        # rospy.loginfo("theta_cam_arr[{}] = {:f}".format(tag.detections[0].id, theta_arr[tag.detections[0].id] * 180 / pi))
    else:   # brake
        rospy.loginfo("no tag detected")
        # initialize & obtain cmd_vel
        cmd_vel = Twist()
        # linear velocity
        w = 0 # no change: if last_tag_id == mid_tag_id
        if last_tag_id == left_tag_id:
            w = -phi
        elif last_tag_id == right_tag_id:
            w = phi
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        # angular velocity
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = w
        # publish w & v
        pub_cmd_vel.publish(cmd_vel) # [uncomment later]
        # theta_cam = 0

        # eliminate the memory of "last_tag_id" once the robot has reached the goal "mid_tag"
        if last_tag_id == mid_tag_id:
            last_tag_id = -4

    # rospy.loginfo("theta_cam: {:f}".format(theta_cam))

    # rospy.loginfo("Theta_cam ={:f}".format(theta_cam * 180 / pi))


'''def turn_around_2pi(w):
    global pub_cmd_vel
    # initialize & obtain cmd_vel
    cmd_vel = Twist()
    # linear velocity
    cmd_vel.linear.x = 0.0 # [reset back to "v" later] / test 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    # angular velocity
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = w

    # publish w & v
    pub_cmd_vel.publish(cmd_vel) # [uncomment later]
'''

'''
def callback_link_states(msg):
    global q_tags_to_world_arr  # Quaternion (april tag0 in the Gazebo World Reference Frame)
    rospy.loginfo(msg)

    # Get quaternions of apriltag tag0 relative to the Gazebo World
    for tag_num in [1, 2]:
        x = msg.pose[tag_num].orientation.x
        y = msg.pose[tag_num].orientation.y
        z = msg.pose[tag_num].orientation.z
        w = msg.pose[tag_num].orientation.w
        q_tags_to_world_arr[tag_num-1] = Quaternion(w,x,y,z)
        # rospy.loginfo("q(apriltag to world){}: x={:.2f}, y={:.2f}, z={:.2f}, w={:.2f}\n".format(tag_num-1, x, y, z, w)) # uncomment later
'''

def main():
    global pub_cmd_vel
    # global pose
    # global q_tag0_to_world       # Quaternion (april tag0 in the Gazebo World Reference Frame)
    # global theta_cam

    rospy.init_node('bearing_controller_gazebo', anonymous=False)

    '''rospy.Subscriber('/gazebo/model_states', ModelStates, callback_link_states)''' # modify global variable "pose"
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback_tag_detections)
    rospy.Subscriber('bearings', Bearings, callback_bearings) # obtain bearings   # important, uncomment later

    pub_cmd_vel = rospy.Publisher("robot_twist", Twist, queue_size = 10)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    finally:
        # "clean up" code: executed on shutdown in case of errors. E.g. closing files/windows
        pass
