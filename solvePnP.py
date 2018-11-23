import numpy as np
import cv2	
from numpy.linalg import inv
import math

fx = 614.1699
fy = 614.9002
cx = 329.9491
cy = 237.2788
camera_mat = np.zeros((3,3,1))
camera_mat[:,:,0] = np.array([[fx, 0, cx], 
		[0, fy, cy],
		[0,  0,  1]])
k1 = 0.1115
k2 = -0.1089
p1 = 0
p2 = 0
dist_coeffs = np.zeros((4,1))
dist_coeffs[:,0] = np.array([[k1, k2, p1, p2]])

# far to close, left to right (order of discovery) in cm
obj_points = np.zeros((6,3,1))
obj_points[:,:,0] = np.array([[00.0, 00.0, 0], 
		[40.5, 00.0, 0],
		[00.0, 28.0, 0],
		[40.5, 28.1, 0],
		[00.0, 59.5, 0],
		[40.5, 58.9, 0]])

img_points = np.zeros((6,2,1))
img_points[:,:,0] = np.array([[119,381], 
		[517,365],
		[172,302],
		[459,290],
		[218,251],
		[430,246]])

retval, rvec, tvec = cv2.solvePnP(obj_points, img_points,camera_mat, dist_coeffs)
print "\nrotationvector\n", rvec, "\n\n", "translationvector\n", tvec,  "\n\n", "rotationmatrix"

rmat = np.zeros((3,3))
cv2.Rodrigues(rvec, rmat, jacobian=0)	
print rmat
rmatinv = inv(rmat)

print "\n", "inverse Rotationmatrix", "\n" 
print rmatinv



print "\n", "translation vector homogenous transform"

transtvec = np.matmul(rmatinv, tvec)
print transtvec

print "\n", "Roll-Pitch-Yaw" 

beta = math.atan2(-rmat[2][0], math.sqrt(rmat[0][0]**2+rmat[1][0]**2))

if beta == math.pi/2:
	alpha = 0
	gamma = math.atan2(rmat[0][1],rmat[1][1])
elif beta == - math.pi/2:
	alpha = 0
	gamma = - math.atan2(rmat[0][1],rmat[1][1])
else:
	alpha = math.atan2(rmat[1][0]/math.cos(beta),rmat[0][0]/math.cos(beta))
	gamma = math.atan2(rmat[2][1]/math.cos(beta), rmat[2][2]/math.cos(beta))
print beta, alpha, gamma, "\n"
