


import numpy as np


# calculate moment of inertia of a 3D box given its dimensions and mass
def moment_of_inertia(mass, x, y, z):
    Ixx = (1/12) * mass * (y**2 + z**2)
    Iyy = (1/12) * mass * (x**2 + z**2)
    Izz = (1/12) * mass * (x**2 + y**2)
    return np.array([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])


# Use parallel axis theorem to calculate moment of inertia of a 3D box
# given its dimensions, mass, and position vector
def moment_of_inertia_parallel_axis(mass, x, y, z, r):
    I = moment_of_inertia(mass, x, y, z)
    Ixx = I[0, 0] + mass * (r[1]**2 + r[2]**2)
    Iyy = I[1, 1] + mass * (r[0]**2 + r[2]**2)
    Izz = I[2, 2] + mass * (r[0]**2 + r[1]**2)
    return np.array([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])


# Calculate moment of inertia for 7 boxes with same mass and dimensions
# but different positions
mass = 0.04 #kg
mass_small = 0.020 #kg

x = 0.008 #m
y = 0.055
z = 0.055
# change to calculate relative to center position vector

r6 = np.array([80, 45, 0])/1000 #top
r1 = np.array([70, 45+64, 0])/1000 #middle
r7 = np.array([60, 45-64, 0])/1000 #bottom

r4 = np.array([70, 45, 64])/1000 #right
r5 = np.array([70, 45, -64])/1000 #left

r2 = np.array([80, 45+64, -64])/1000 #top right
r3 = np.array([60 , 45-64, 64])/1000 #bottom left
I1 = moment_of_inertia_parallel_axis(mass, x, y, z, r1)
I2 = moment_of_inertia_parallel_axis(mass, x, y, z, r2)
I3 = moment_of_inertia_parallel_axis(mass, x, y, z, r3)
I4 = moment_of_inertia_parallel_axis(mass, x, y, z, r4)
I5 = moment_of_inertia_parallel_axis(mass, x, y, z, r5)
I6 = moment_of_inertia_parallel_axis(mass_small, x, y, z, r6)
I7 = moment_of_inertia_parallel_axis(mass_small, x, y, z, r7)
print(I1)
print(I2)
print(I3)
print(I4)
print(I5)
print(I6)
print(I7)

print(I1+I2+I3+I4+I5+I6+I7)


print(5*mass + 2*mass_small)

# print total mass, center of mass, and total intertia tensor
tool_mass = 5*mass + 2*mass_small
r = (mass*r1 + mass_small*r2 + mass_small*r3 + mass*r4 + mass*r5 + mass*r6 + mass*r7)/tool_mass
I = I1 + I2 + I3 + I4 + I5 + I6 + I7 #+ mass1*(r1 - r).dot(r1 - r) + mass2*(r2 - r).dot(r2 - r) + mass3*(r3 - r).dot(r3 - r) + mass4*(r4 - r).dot(r4 - r) + mass5*(r5 - r).dot(r5 - r) + mass6*(r6 - r).dot(r6 - r) + mass7*(r7 - r).dot(r7 - r)
print('mass =', mass, 'kg')
print('r =', r, 'm')
print('I =', I, 'kg m^2')


"""
Default data:
Mass
0.73
Flange to Center of Mass of Load Vector
-0.01, 0 , 0.03

Inertia Tensor
0.001, 0, 0
0, 0.0025, 0
0, 0, 0.0017

Transformation Matrix from Flange to End-Effector
0.7001, 0.7071, 0.0000 0.0000
-0.7071, 0.7001, 0.0000 0.0000
0.0000, 0.0000, 1.0000 0.0000
0.0000, 0.0000, 0.0000 1.0000
"""

robot_mass = 0.73
robot_r = np.array([-0.01, 0, 0.03])
robot_I = np.array([[0.001, 0, 0], [0, 0.0025, 0], [0, 0, 0.0017]])
robot_T = np.array([[0.7001, 0.7071, 0.0000, 0.0000], [-0.7071, 0.7001, 0.0000, 0.0000], [0.0000, 0.0000, 1.0000, 0.0000], [0.0000, 0.0000, 0.0000, 1.0000]])


# Print final mass, center of mass, and total intertia tensor
total_mass = robot_mass + tool_mass
r = (robot_mass*robot_r + tool_mass*r)/(tool_mass + robot_mass)
I = robot_I + I1 + I2 + I3 + I4 + I5 + I6 + I7 #+ mass1*(r1 - r).dot(r1 - r) + mass2*(r2 - r).dot(r2 - r) + mass3*(r3 - r).dot(r3 - r) + mass4*(r4 - r).dot(r4 - r) + mass5*(r5 - r).dot(r5 - r) + mass6*(r6 - r).dot(r6 - r) + mass7*(r7 - r).dot(r7 - r)
print('mass =', total_mass, 'kg')
print('r =', r, 'm')
print('I =', I, 'kg m^2')

