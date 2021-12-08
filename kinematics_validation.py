# Pratik Acharya
# UID : 117513615
# To run the program use the following command on a terminal window on Ubuntu
# python3 homework5.py

from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# Declaring the variable for joint angles
theta1, theta2, theta4, theta5, theta6, theta7 = symbols("theta1 theta2 theta4 theta5 theta6 theta7")
d1, d3, d5, d7 = symbols("d1 d3 d5 d7")
d1 = 300
d3 = 300
d5 = 300
d7 = 300


transformationMatrix01 = Matrix([[cos(theta1), 0, sin(theta1), 300*cos(theta1)], [sin(theta1), 0, -cos(theta1), 300*sin(theta1)],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])
transformationMatrix12 = Matrix([[cos(theta2), 0, sin(theta2), 300*cos(theta2)], [sin(theta2), 0, -cos(theta2), 300*sin(theta2)],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])
transformationMatrix23 = Matrix([[cos(0), 0, sin(0), 0], [sin(0), 0, -cos(0), 0],
                                 [0, 1, 0, d3], [0, 0, 0, 1]])
transformationMatrix34 = Matrix([[cos(theta4), 0, sin(theta4), 300*cos(theta4)], [sin(theta4), 0, -cos(theta4), 300*sin(theta4)],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])
transformationMatrix45 = Matrix([[cos(theta5), 0, sin(theta5), 300*cos(theta5)], [sin(theta5), 0, -cos(theta5), 300*sin(theta5)],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])
transformationMatrix56 = Matrix([[cos(theta6), 0, sin(theta6), 300*cos(theta6)], [sin(theta6), 0, -cos(theta6), 300*sin(theta6)],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])
transformationMatrix67 = Matrix([[cos(theta7), 0, sin(theta7), 300*cos(theta7)], [sin(theta7), 0, -cos(theta7), 300*sin(theta7)],
                                 [0, 1, 0, 0], [0, 0, 0, 1]])

# Calculating transformation matrix from the base frame to all frames frame
transformationMatrix02 = transformationMatrix01 * transformationMatrix12
transformationMatrix04 = transformationMatrix01 * transformationMatrix12 * transformationMatrix23 * \
                         transformationMatrix34
transformationMatrix05 = transformationMatrix01 * transformationMatrix12 * transformationMatrix23 * \
                         transformationMatrix34 * transformationMatrix45
transformationMatrix06 = transformationMatrix01 * transformationMatrix12 * transformationMatrix23 * \
                         transformationMatrix34 * transformationMatrix45 * transformationMatrix56
transformationMatrix07 = transformationMatrix01 * transformationMatrix12 * transformationMatrix23 * \
                         transformationMatrix34 * transformationMatrix45 * transformationMatrix56 * \
                         transformationMatrix67

# creating a matrix to use for jacobian
theta_mat = Matrix([theta1, theta2, theta4, theta5, theta6, theta7])

# Isolating the z column from the transformation matrices
z_1 = transformationMatrix01[:3, 2]
z_2 = transformationMatrix02[:3, 2]
z_3 = transformationMatrix04[:3, 2]
z_4 = transformationMatrix05[:3, 2]
z_5 = transformationMatrix06[:3, 2]
z_6 = transformationMatrix07[:3, 2]

# Isolating the position vector from transformation matrix
xp = transformationMatrix07[:3, 3]

# Calculating the Jacobian
dh_q = xp.jacobian(theta_mat)

z_matrix = z_1.row_join(z_2).row_join(z_3).row_join(z_4).row_join(z_5).row_join(z_6)
z_matrix_col = Matrix([[z_1[2]/2], [(z_1[2]+z_2[2])/2], [(z_1[2]+z_2[2]+z_3[2])/2], [(z_1[2]+z_2[2]+z_3[2]+z_4[2])/2],
                       [(z_1[2]+z_2[2]+z_3[2]+z_4[2]+z_5[2])/2], [(z_1[2]+z_2[2]+z_3[2]+z_4[2]+z_5[2]+z_6[2])/2]])



# Calculation the Jacobian matrix for velocity kinematics
J_0 = dh_q.col_join(z_matrix)

# Substituting the values of joint angles in the jacobian
# q_new = Matrix([[0.00001], [0.0001], [0.0001], [0.0001], [0.00001], [0.0001]])
q_new = Matrix([[0], [-pi/12], [-pi/8], [-pi/6], [-pi/5], [-pi/4]])

# External Force vector
# force = Matrix([[0], [-5], [0], [0], [0], [0]])

# Mass of links
# mass = Matrix([[3.95], [4.5], [-5.07], [3.41], [3.38], [0.35]])

# gravity = 9.8

# potential_e = gravity*mass.transpose()*z_matrix_col
# g_q = potential_e.jacobian(theta_mat).transpose()

# Finding the value of Jacobian matrix after substitution
subs_m = N(J_0.subs([(theta1, q_new[0]), (theta2, q_new[1]), (theta4, q_new[2]), (theta5, q_new[3]),
                     (theta6, q_new[4]), (theta7, q_new[5])]), 5)

# Calculating the Jacobian Inverse
pprint(subs_m)
J_0_inv = N(subs_m.inv(), 5)

# Creating vectors to store the value of the velocities and positions that are calculated from the Inverse Jacobian
x_dot = []
z_dot = []
q_dot = []

# Setting theta to calculate the value of velocities ar each point on the circle
th = pi/2

# Initializing the plot
fig = plt.figure()
ax = Axes3D(fig)
ax.set_ylim3d(-150, 150)
ax.set_zlim3d(350, 650)
ax.set_xlim3d(-150, 150)

# fig_t, axs = plt.subplots(6, sharex=True)
# fig_t.text(0.01, 0.5, 'Torque', va='center', rotation='vertical')
# fig_t.tight_layout(pad=0.5)
# axs[5].set_xlabel("Time")
# axs[0].set_xlim([0, 200])
# axs[1].set_xlim([0, 200])
# axs[2].set_xlim([0, 200])
# axs[3].set_xlim([0, 200])
# axs[4].set_xlim([0, 200])
# axs[5].set_xlim([0, 200])
# axs[0].set_xlabel("Link 1")
# axs[1].set_xlabel("Link 2")
# axs[2].set_xlabel("Link 3")
# axs[3].set_xlabel("Link 4")
# axs[4].set_xlabel("Link 5")
# axs[5].set_xlabel("Link 6")

# Calculating the velocities at 40 points on the circle
for i in range(40):
    x_dot.append(N(((-1)*1.25*150*sin(th)), 5))
    z_dot.append(N((150*1.25*cos(th)), 5))
    th += pi/20

# Iterating through each point to get the value of the joint velocities and using the forward position kinematics
# transform to get the values of the end effector position

# torque_time = 0

for i in range(40):
    # Matrix to save the velocity matrix
    j_vel_mat = Matrix([[x_dot[i]], [z_dot[i]], [0], [0], [0], [0]])
    # Saving the joint velocities calculated from the inverse jacobian

    q_dot.append(J_0_inv*j_vel_mat)
    # Getting the updated joint angles
    q_new = (q_new + q_dot[i]*0.05).evalf()

    # Putting the values through the transformation matrix
    transform = N(transformationMatrix07.subs([(theta1, q_new[0]), (theta2, q_new[1]), (theta4, q_new[2]),
                                               (theta5, q_new[3]), (theta6, q_new[4]), (theta7, q_new[5])]), 5)

    transform_x = int(transform[0, 3])  - 1550
    transform_y = int(transform[1, 3]) + 230
    transform_z = int(transform[2, 3]) + 1000
    # Substituting the new joint values into the jacobian
    subs_m = N(J_0.subs([(theta1, q_new[0]), (theta2, q_new[1]), (theta4, q_new[2]), (theta5, q_new[3]),
                         (theta6, q_new[4]), (theta7, q_new[5])]), 5)
    # Calculation the jacobian inverse
    J_0_inv = N(subs_m.inv(), 5)

    # Calculating the external force
    # external_f = subs_m.transpose()*force

    # Calculating the torque
    # torque = N(g_q.subs([(theta1, q_new[0]), (theta2, q_new[1]), (theta4, q_new[2]),
    #                                   (theta5, q_new[3]), (theta6, q_new[4]), (theta7, q_new[5])]) - external_f , 5)
    # pprint(torque)
    # Plotting the point on the graph
    ax.scatter3D(transform_x, transform_y, transform_z, c="b")
    # axs[0].scatter(torque_time, torque[0], c="r", marker=".")
    # axs[1].scatter(torque_time, torque[1], c="r", marker=".")
    # axs[2].scatter(torque_time, torque[2], c="r", marker=".")
    # axs[3].scatter(torque_time, torque[3], c="r", marker=".")
    # axs[4].scatter(torque_time, torque[4], c="r", marker=".")
    # axs[5].scatter(torque_time, torque[5], c="r", marker=".")
    plt.pause(0.05)

    # torque_time += 5

# Showing the plot
plt.show()
