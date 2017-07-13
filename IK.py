from sympy import symbols, sin, cos, tan, pi, simplify, atan2, sqrt, acos
from sympy.matrices import Matrix

#Now, for the intrinsic rotations of the gripper axis,
# as it is different in our DH convention and the URDF file.
R_z = Matrix([[    cos(pi),   -sin(pi),          0],
	         [    sin(pi),    cos(pi),          0],
	         [          0,          0,          1]])

R_y = Matrix([[ cos(-pi/2),          0, sin(-pi/2)],
	         [          0,          1,          0],
	         [-sin(-pi/2),          0, cos(-pi/2)]])

#For Intrinsic Rotations:
R_corr = simplify(R_z * R_y)

# Defining the roll, pitch, yaw angles.
roll, pitch, yaw =  0, 0, 0

#Defining the rotation matrices.
R_z_yaw = Matrix([[    cos(yaw),   -sin(yaw),          0],
	             [    sin(yaw),    cos(yaw),          0],
	             [           0,           0,          1]])

R_y_pitch = Matrix([[ cos(pitch),          0, sin(pitch)],
	               [          0,          1,          0],
	               [-sin(pitch),          0, cos(pitch)]])

R_x_roll = Matrix([[           1,          0,          0],
	              [           0,  cos(roll), -sin(roll)],
	              [           0,  sin(roll),  cos(roll)]])

Rrpy = R_z_yaw * R_y_pitch * R_x_roll

###### Solving for the WC's. ######
#Defining the necessary symbols.
px, py, pz, dG = symbols("px, py, pz, dG")
 
WC_sub = {px:2.153 ,py:0.000 ,pz:1.946 #Defining the end effector location.
          ,dG:0.303 }#Same as d7, from the DH parameter table.

p_loc_vector = Matrix([px, py, pz]) #Column vector containing the end effector location.

dG_vector = Matrix([0, 0, dG]) #Cloumn vector defining the end effector as a location vector
# relative to the Wrist Coordinates.

#Equation for getting the World Cordinates as a column vector.
WC_loc_vector = p_loc_vector - Rrpy*R_corr.inv()*dG_vector

#Defining the wrist coordinates from the column vector made above.
WC_x = WC_loc_vector[0].evalf(subs = WC_sub)
WC_y = WC_loc_vector[1].evalf(subs = WC_sub)
WC_z = WC_loc_vector[2].evalf(subs = WC_sub)

print("The Wrist Coordinates in x, y, z = ", WC_x, WC_y, WC_z)

###### Solving for the first three angles. ######
#Defining the symbols used in the DH parameters.
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

#Dictionary for the DH parameters.
s = {alpha0:     0,  a0:     0,  d1:  0.75,
     alpha1: -pi/2,  a1:  0.35,  d2:     0,  q2:q2-pi/2, 
     alpha2:     0,  a2:  1.25,  d3:     0,
     alpha3: -pi/2,  a3:-0.054,  d4:  1.50,
     alpha4:  pi/2,  a4:     0,  d5:     0,
     alpha5: -pi/2,  a5:     0,  d6:     0,
     alpha6:     0,  a6:     0,  d7: 0.303,  q7:0}

#Calculating Theta1, for the first joint.
theta1 = atan2(WC_y,WC_x)
theta1 = theta1.evalf(subs=s)

#Calculating Theta2, for the second joint.
#Refer to diagram for the new symbols.
l0 = sqrt(WC_x**2 + WC_y**2)
beta1 = atan2(WC_z-d1, l0-a1)
l1 = sqrt((l0-a1)**2 + (WC_z-d1)**2)
l2 = sqrt(a3**2 + d4**2)
cos_beta2 = (a2**2 + l1**2 - l2**2)/(2*a2*l1) #Using the Law of Cosines.
beta2 = atan2(sqrt(1-cos_beta2**2),cos_beta2) #Using the sin, cos Pythagorean Identity.
theta2 = pi/2 - beta1 - beta2
theta2 = theta2.evalf(subs=s)

#Calculating Theta3, for the third joint.
beta0 = atan2(-a3,d4)
cos_beta3 = (a2**2 + l2**2 - l1**2)/(2*a2*l2) #Using the Law of Cosines.
beta3 = atan2(sqrt(1-cos_beta3**2),cos_beta3) #Using the sin, cos Pythagorean Identity.
theta3 = pi/2 - beta3 - beta0
theta3 = theta3.evalf(subs=s)

print("theta1,theta2,theta3 = ",theta1, theta2, theta3)

### Finding the last three angles ###

#R0_6 = Rrpy * R_corr.inv()
#R0_3 * R3_6 = Rrpy * R_corr.inv()
#Multiplying both sides with the inverse of R0_3
#R3_6 = R0_3.inv() * Rrpy * R_corr.inv()

#So now we need R0_3.
#Defining the individual rotation matrices.
R0_1 = Matrix([[             cos(q1),            -sin(q1),            0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0)],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0)]])
R0_1 = R0_1.subs(s)

R1_2 = Matrix([[             cos(q2),            -sin(q2),            0],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1)],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1)]])
R1_2 = R1_2.subs(s)

R2_3 = Matrix([[             cos(q3),            -sin(q3),            0],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2)],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2)]])
R2_3 = R2_3.subs(s)

#R6_G will be an identity matrix as there is only a translation component between frame 6
# and the gripper frame so there is no need to define that one.

R0_2 = simplify(R0_1 * R1_2) 
R0_3 = simplify(R0_2 * R2_3)

#Substituing in the known values of the first three angles.

R3_6 = R0_3.inv() * Rrpy * R_corr.inv()
R3_6 = R3_6.evalf(subs={q1:theta1,q2:theta2,q3:theta3})

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

print("theta4, theta5, theta6 = ", q4, q5, q6)