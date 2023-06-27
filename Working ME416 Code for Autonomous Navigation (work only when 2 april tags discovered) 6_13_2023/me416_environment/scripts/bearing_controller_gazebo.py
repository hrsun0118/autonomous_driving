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

theta_cam = 0
q_tags_to_world_arr = np.asarray(2 * [Quaternion(1,0,0,0)])

def callback_bearings(msg):
    """Callback to receive a message from bearings"""
    global pub_cmd_vel
    global pose
    global theta_cam

    # obtain u
    beta0_star = np.asarray([[-1], [0]])    # modification: should be  [[0], [-1]] - switch aprial tag frame (x,z) to (z,x) to match w/ Gazebo world frame
    beta1_star = np.asarray([[1], [0]])
    # u_bar = np.asarray([[0],[1.75]]) # a feedforward term [1.6 - 1.9] (here = e_dot, which is speed error) (added to help the turtle goes through the gate with speed)
    u = (np.asarray([[msg.beta0.x],[msg.beta0.y]]) \
        + np.asarray([[msg.beta1.x],[msg.beta1.y]])) \
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
        sin_theta_u = u[1][0] / abs_u   # Q: 2 answers for sin_theta_u:
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

    # set phi = 0.01 & then adjust
    phi = 0.01
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

    # # log info    # [uncomment later]
    # rospy.loginfo("cmd/vel:")
    # rospy.loginfo("Linear: x = %.3f, y = %.3f, z = %.3f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z)  # uncomment later
    # rospy.loginfo("Angular: x = %.3f, y = %.3f, z = %.3f",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z)  # uncomment later

def callback_tag_detections(tag):
    global pose
    global q_tags_to_world_arr       # Quaternion (april tag0 in the Gazebo World Reference Frame)
    global theta_cam

    # Get quaternions of camera relative to the apriltag tag0
    theta_arr = np.zeros(2)
    # theta_cam_to_tag_arr = np.zeros(2)
    # rospy.loginfo("new tag:")
    for element in tag.detections:
        # get quaternions of camera relative to the apriltagss
        x = element.pose.pose.pose.orientation.x
        y = element.pose.pose.pose.orientation.y
        z = element.pose.pose.pose.orientation.z
        w = element.pose.pose.pose.orientation.w
        q_cam_to_tags = Quaternion(w, x, y, z)                      # get Quaternion of camera to tag
        # rospy.loginfo("tag[{}] = {:f}, {:f}, {:f}, {:f}".format(element.id, w, x, y, z))
        theta_cam_to_tag = q_cam_to_tags.to_euler(degrees=False)    # get euler's angle of camera to tag
        rospy.loginfo("Euler's angle [{}]: {:f}, {:f}, {:f}".format(element.id,theta_cam_to_tag[0], theta_cam_to_tag[1], theta_cam_to_tag[2]))
        theta_arr[element.id] = theta_cam_to_tag[1] + pi / 2                 # get the 2nd angle (index = 1) - confirmed: it's rotating around the y axis

        # # get quaternions of camera relative to the world: using quaternions multiplication
        # q_tags_to_world = Quaternion(q_tags_to_world_arr[element.id][0], q_tags_to_world_arr[element.id][1], q_tags_to_world_arr[element.id][2], q_tags_to_world_arr[element.id][3])
        # q_cam_to_world = q_cam_to_tags * q_tags_to_world
        # # get Euler's Angles of each tag
        # theta_arr[element.id] = q_cam_to_world.to_euler(degrees=False)[1] # to_euler(): return as a tuple: (roll, pitch, yaw): rotation around (x, y, z) axis, get index = 1 (2nd element)
        #
        # rospy.loginfo("theta(cam to apriltag tag{}): e1: {:.2f}, e2: {:.2f}, e3: {:.2f}".format(element.id, theta_cam_to_tag[0], theta_cam_to_tag[1],  theta_cam_to_tag[2]))

    # Get theta_cam
    if len(tag.detections) == 2:
        theta_cam = (theta_arr[0] + theta_arr[1])/2
        # rospy.loginfo("theta_cam_arr[0] ={:f}, theta_cam_arr[1] ={:f}".format(theta_arr[0] * 180 / pi, theta_arr[1] * 180 / pi))
    # elif len(tag.detections) == 1:
    #     theta_cam = theta_arr[tag.detections[0].id]
        # rospy.loginfo("theta_cam_arr[{}] = {:f}".format(tag.detections[0].id, theta_arr[tag.detections[0].id] * 180 / pi))
    # elif len(tag.detections) == 0:
    #     rospy.loginfo("no tag detected")
    else:
        theta_cam = 0

    # rospy.loginfo("theta_cam: {:f}".format(theta_cam))

    # rospy.loginfo("Theta_cam ={:f}".format(theta_cam * 180 / pi))

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
