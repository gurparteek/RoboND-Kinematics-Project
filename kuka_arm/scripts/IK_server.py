#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

#Solving WC coordinates from the equation [WC_x,WC_y,WC_z] = [p_x,p_y,p_z] - R0_6 * [0,0,d7]
# where R0_6 is Rrpy * R_corr.inv() as the gripper frame that gives
# us the pose and the URDF frame are not aligned.

#First, we need a correction matrix.

#Now, for the intrinsic rotations of the gripper axis,
# as it is different in our DH convention and the URDF file.
# R_z = Matrix([[    cos(pi),   -sin(pi),          0],
#              [    sin(pi),    cos(pi),          0],
#              [          0,          0,          1]])

# R_y = Matrix([[ cos(-pi/2),          0, sin(-pi/2)],
#              [          0,          1,          0],
#              [-sin(-pi/2),          0, cos(-pi/2)]])

# #For Intrinsic Rotations:
# R_corr = R_z * R_y
# R_corr = R_corr.evalf()
# R_corr_inv = R_corr.inv()
# R_corr_inv = simplify(R_corr_inv)
# print R_corr_inv

R_corr_inv = Matrix([[  0,    0, 1.0],
                     [  0, -1.0,   0],
                     [1.0,    0,   0]])

#Defining symbolic roll, pitch and yaw variables to enable global Rrpy calculation outside the loop.
roll_sym, pitch_sym, yaw_sym = symbols("roll_sym, pitch_sym, yaw_sym")

#Defining the rotation matrices.
# R_z_yaw = Matrix([[    cos(yaw_sym),   -sin(yaw_sym),          0],
#                  [    sin(yaw_sym),    cos(yaw_sym),          0],
#                  [           0,           0,          1]])

# R_y_pitch = Matrix([[ cos(pitch_sym),          0, sin(pitch_sym)],
#                    [          0,          1,          0],
#                    [-sin(pitch_sym),          0, cos(pitch_sym)]])

# R_x_roll = Matrix([[           1,          0,          0],
#                   [           0,  cos(roll_sym), -sin(roll_sym)],
#                   [           0,  sin(roll_sym),  cos(roll_sym)]])

# Rrpy_global = R_z_yaw * R_y_pitch * R_x_roll
# Rrpy_global = simplify(Rrpy_global)
# print Rrpy_global

Rrpy_global = Matrix([[cos(pitch_sym)*cos(yaw_sym), sin(pitch_sym)*sin(roll_sym)*cos(yaw_sym) - sin(yaw_sym)*cos(roll_sym), sin(pitch_sym)*cos(roll_sym)*cos(yaw_sym) + sin(roll_sym)*sin(yaw_sym)],
                      [sin(yaw_sym)*cos(pitch_sym), sin(pitch_sym)*sin(roll_sym)*sin(yaw_sym) + cos(roll_sym)*cos(yaw_sym), sin(pitch_sym)*sin(yaw_sym)*cos(roll_sym) - sin(roll_sym)*cos(yaw_sym)],
                      [            -sin(pitch_sym),                                           sin(roll_sym)*cos(pitch_sym),                                           cos(pitch_sym)*cos(roll_sym)]])



def handle_calculate_IK(req):
    global R_corr_inv, Rrpy_global

    #Defining DH parameter symbols.
    alpha0, alpha1, alpha2 = symbols('alpha0:3')
    a0, a1, a2, a3 = symbols('a0:4')
    d1, d2, d3, d4 = symbols('d1:5')
    d7 = symbols('d7') #Define the offset between the wrist center and the end effector.

    #Defining joint angle symbols.
    q1, q2, q3= symbols('q1:4')

    #Dictionary for the DH parameters.
    s = {alpha0:     0,  a0:     0,  d1:  0.75,
         alpha1: -pi/2,  a1:  0.35,  d2:     0,  q2:q2-pi/2, #Defining the 90 degree offset.
         alpha2:     0,  a2:  1.25,  d3:     0,
                         a3:-0.054,  d4:  1.50,
                                     d7: 0.303}

    #Defining the individual rotation matrices.
    # R0_1 = Matrix([[             cos(q1),            -sin(q1),            0],
    #                [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0)],
    #                [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0)]])
    # R0_1 = R0_1.subs(s)

    # R1_2 = Matrix([[             cos(q2),            -sin(q2),            0],
    #                [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1)],
    #                [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1)]])
    # R1_2 = R1_2.subs(s)

    # R2_3 = Matrix([[             cos(q3),            -sin(q3),            0],
    #                [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2)],
    #                [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2)]])
    # R2_3 = R2_3.subs(s)

    # R0_2 = simplify(R0_1 * R1_2) 
    # R0_3 = simplify(R0_2 * R2_3)

    # R0_3_inv_main = R0_3.inv()
    # print R0_3_inv_main

    R0_3_inv_main = Matrix([[(sin(q1)*cos(q2 + q3)**2/(sin(q2 + q3)*cos(q1)) - sin(q1)/(sin(q2 + q3)*cos(q1)))*sin(q1) - cos(q2 + q3)**2/(sin(q2 + q3)*cos(q1)) + 1/(sin(q2 + q3)*cos(q1)), -(sin(q1)*cos(q2 + q3)**2/(sin(q2 + q3)*cos(q1)) - sin(q1)/(sin(q2 + q3)*cos(q1)))*cos(q1),  cos(q2 + q3)],
                            [                                                                                                      -sin(q1)**2*cos(q2 + q3)/cos(q1) + cos(q2 + q3)/cos(q1),                                                                       sin(q1)*cos(q2 + q3), -sin(q2 + q3)],
                            [                                                                                                                                                     -sin(q1),                                                                                    cos(q1),             0]])

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        eef_pose_count = 0
        for x in xrange(0, len(req.poses)):
            eef_pose_count += 1
            print "Calculating IK for eef pose # ", eef_pose_count
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ###### Solving for the Wrist Center Coordinates's. ######
            
            #Solving WC coordinates from the equation [WC_x,WC_y,WC_z] = [p_x,p_y,p_z] - R0_6 * [0,0,d7]
            # where R0_6 is Rrpy * R_corr.inv() as the gripper frame that gives
            # us the pose and the URDF frame are not aligned.

            #The correction matrix, R_corr is already calculated.

            #Rrpy has been calculated symbolically outside the loop.
            #Substituing the actual values of roll, pitch and yaw.
            Rrpy = Rrpy_global.evalf(subs={roll_sym:roll, pitch_sym:pitch, yaw_sym:yaw})

            #Defining the location column vectors.

            p_loc_vector = Matrix([px, py, pz]) #Column vector containing the end effector location.

            dG_vector = Matrix([0, 0, d7]) #Cloumn vector defining the end effector as a location vector
            # relative to the Wrist Coordinates.

            #Equation for getting the World Cordinates as a column vector.
            #Need to perform: WC_loc_vector = p_loc_vector - Rrpy*R_corr_inv*dG_vector
            Rrpy_corr = Rrpy*R_corr_inv
            dG_vector_rotated = Rrpy_corr*dG_vector
            WC_loc_vector = p_loc_vector - dG_vector_rotated

            #Defining the wrist coordinates from the column vector made above.
            WC_x = WC_loc_vector[0]
            WC_y = WC_loc_vector[1]
            WC_z = WC_loc_vector[2]
            
            ######Calculating Theta1, for the first joint.######

            theta1 = atan2(WC_y,WC_x)
            theta1 = theta1.evalf(subs=s)

            ######Calculating Theta2, for the second joint.######

            #Refer to diagram for the new variables used.
            l0 = sqrt(WC_x**2 + WC_y**2)
            beta1 = atan2(WC_z-d1, l0-a1)
            l1 = sqrt((l0-a1)**2 + (WC_z-d1)**2)
            l2 = sqrt(a3**2 + d4**2)
            cos_beta2 = (a2**2 + l1**2 - l2**2)/(2*a2*l1) #Using the Law of Cosines.
            beta2 = atan2(sqrt(1-cos_beta2**2),cos_beta2) #Using the sin, cos Pythagorean Identity.
            theta2 = pi/2 - beta1 - beta2
            theta2 = theta2.evalf(subs=s)

            ######Calculating Theta3, for the third joint.######

            beta0 = atan2(-a3,d4)
            cos_beta3 = (a2**2 + l2**2 - l1**2)/(2*a2*l2) #Using the Law of Cosines.
            beta3 = atan2(sqrt(1-cos_beta3**2),cos_beta3) #Using the sin, cos Pythagorean Identity.
            theta3 = pi/2 - beta3 - beta0
            theta3 = theta3.evalf(subs=s)

            ###### Finding the last three angles ######

            #R0_6 = Rrpy * R_corr.inv()
            #R0_3 * R3_6 = Rrpy * R_corr.inv()
            #Multiplying both sides with the inverse of R0_3
            #R3_6 = R0_3.inv() * Rrpy * R_corr.inv()

            #So now we need R0_3 and this has already been defined symbolically outside the loop.

            #Need to perform: R3_6 = R0_3_inv * Rrpy * R_corr_inv
            # Rrpy * R_corr_inv = Rrpy_corr calculated above.
            R0_3_inv = R0_3_inv_main.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
            R3_6 = R0_3_inv * Rrpy_corr
            R3_6.evalf()

            #Substituing in the known values of the first three angles.

            #R3_6 = R3_6

            ###### Now solving for the Euler angles from R3_6 ######

            ###### Getting the symbolic R0_3 ######
            #Uncommenting the following gives us the symbolic R3_6 to do calculations for finding
            # Euler angles from a matrix.
            # R3_4 = Matrix([[             cos(q4),            -sin(q4),            0],
            #                [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3)],
            #                [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3)]])
            # R3_4 = R3_4.subs(s)

            # R4_5 = Matrix([[             cos(q5),            -sin(q5),            0],
            #                [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4)],
            #                [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4)]])
            # R4_5 = R4_5.subs(s)

            # R5_6 = Matrix([[             cos(q6),            -sin(q6),            0],
            #                [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5)],
            #                [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5)]])
            # R5_6 = R5_6.subs(s)

            # R3_5 = simplify(R3_4 * R4_5) 
            # R3_6_sym = simplify(R3_5 * R5_6)

            #The symbolic R3_6 comes out to be:
            #[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
            #[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
            #[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

            q4 = atan2(R3_6[2,2],-R3_6[0,2])
            q5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            q6 = atan2(-R3_6[1,1],R3_6[1,0])

            theta4, theta5, theta6 = q4,q5,q6
            print "Done calculating IK for eef pose # ", eef_pose_count

            # Populate response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
