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
a1,a2,a3,d1,d4 = symbols('a1,a2,a3,d1,d4')
sub={a1:0.35, a2:1.25, a3:0.054, d1:0.75, d4:1.5}

#Calculating Theta1, for the first joint.
theta1 = atan2(WC_y,WC_x)

#Calculating Theta2, for the second joint.
#Refer to diagram for the new symbols.
l0 = sqrt(WC_x**2 + WC_y**2)
beta1 = atan2(WC_z-d1, l0-a1)
l1 = sqrt((l0-a1)**2 + (WC_z-d1)**2)
l2 = sqrt(a3**2 + d4**2)
cos_beta2 = (a2**2 + l1**2 - l2**2)/(2*a2*l1) #Using the Law of Cosines.
beta2 = atan2(sqrt(1-cos_beta2**2),cos_beta2) #Using the sin, cos Pythagorean Identity.
theta2 = pi/2 - beta1 - beta2

#Calculating Theta3, for the third joint.
beta0 = atan2(a3,d4)
cos_beta3 = (a2**2 + l2**2 - l1**2)/(2*a2*l2) #Using the Law of Cosines.
beta3 = atan2(sqrt(1-cos_beta3**2),cos_beta3) #Using the sin, cos Pythagorean Identity.
theta3 = pi/2 - beta3 - beta0

print("theta1,theta2,theta3 = ",(theta1).evalf(subs=sub), (theta2).evalf(subs=sub), (theta3).evalf(subs=sub))