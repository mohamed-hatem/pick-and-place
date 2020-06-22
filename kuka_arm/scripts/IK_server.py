#!/usr/bin/env python

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
def rotx(q):
        rx = Matrix([[ 1,      0,       0],
                      [ 0, cos(q), -sin(q)],
                      [ 0, sin(q),  cos(q)]])
	return rx
    
    
def roty(q):
	ry = Matrix([[  cos(q), 0, sin(q)],
                      [       0, 1,      0],
                      [ -sin(q), 0, cos(q)]])
	return ry
    
    
def rotz(q):
	rz = Matrix([[ cos(q), -sin(q), 0],
                      [ sin(q),  cos(q), 0],
                      [      0,       0, 1]])
	return rz

def transform_matrix(alpha, a, d, q):
	TF = Matrix([[             cos(q),            -sin(q),            0,              a],
	 	[ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
		[ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
		[                   0,                   0,            0,               1]])
	return TF


def handle_calculate_IK(req):
    rospy.loginfo("Received %s end_effector_pointf-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') 
	# Modified DH parameters
		dh = {alpha0:       0, a0:      0, d1:  0.75, q1:      q1,
    	    alpha1:    -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
     	    alpha2:        0, a2:   1.25, d3:     0, q3:	  q3,
     	    alpha3:    -pi/2, a3: -0.054, d4:   1.5, q4:	  q1,
     	    alpha4:     pi/2, a4:      0, d5:     0, q5:	  q5,
     	    alpha5:    -pi/2, a5:      0, d6:     0, q6:	  q6,
     	    alpha6:        0, a6:      0, d7: 0.303, q7:      0}

		rtd = 180 / pi
		dtr = pi / 180
	# Create individual transformation matrices
		T0_1=transform_matrix(alpha0, a0, d1, q1).subs(dh)
		T1_2=transform_matrix(alpha1, a1, d2, q2).subs(dh)
		T2_3=transform_matrix(alpha2, a2, d3, q3).subs(dh)
    	T3_4=transform_matrix(alpha3, a3, d4, q4).subs(dh)
    	T4_5=transform_matrix(alpha4, a4, d5, q5).subs(dh)
    	T5_6=transform_matrix(alpha5, a5, d6, q6).subs(dh)
    	T6_G=transform_matrix(alpha6, a6, d7, q7).subs(dh)

    	T0_G=T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G



        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
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
     
			r, p, y = symbols('r p y')
   	    #accounting for the difference between urdf and gazebo
			rotation_correction=rotz(180*dtr)*roty(-90*dtr)
			#preform extrinsic rotation and apply correction
			end_effector_rotation = rotz(yaw)*roty(pitch)*rotx(roll)*rotation_correction

			end_effector_point=Matrix([[px],[py],[pz]])

    	    wrist_center_coord= end_effector_point - (0.303)*end_effector_rotation[:,2]  
	    #
	    # Calculate joint angles using Geometric IK method
			theta1=atan2(wrist_center_coord[1],wrist_center_coord[0])
			sideA=1.50
			sideBXY=sqrt(wrist_center_coord[0]*wrist_center_coord[0]+wrist_center_coord[1]*wrist_center_coord[1])-0.35 
			sideBZ=wrist_center_coord[2]-0.75
			sideB=sqrt(pow((sideBXY),2) + pow((sideBZ), 2))
			sideC=1.25
    	    #cosine law
    	    angleA=acos((   sideB * sideB + sideC * sideC - sideA * sideA )/( 2 * sideB * sideC ))
    	    angleB=acos((   sideC * sideC + sideA * sideA - sideB * sideB )/( 2 * sideA * sideC ))
    	    angleC=acos((   sideB * sideB + sideA * sideA - sideC * sideC)/( 2 * sideA * sideB ))
    	    ### Thetas
    	    theta2= pi/2 - angleA - atan2(sideBZ, sideBXY)
    	    theta3= pi/2 - angleB + 0.036 #sag in link 4
    	    #we can get rotation transfomation from base to joint 3 by multiplying the transform matricies associated 
			R0_3 = T0_1[0:3, 0:3]*T1_2[0:3, 0:3]*T2_3[0:3, 0:3]
    	    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    	    R3_6 = R0_3.T * end_effector_rotation
  

    	    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
    	    theta5 = atan2(sqrt(R3_6[0, 2]**2+R3_6[2, 2]**2), R3_6[1, 2])
    	    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

    	    # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    #
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
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

