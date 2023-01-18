'''TransformHelpers.py

   These are helper functions for rotation and transform matrices.

   They simply convert the information between different formats.  For
   example python lists or NumPy arrays, encoding Euler Angles,
   Quaternions, or Rotation Matrices.  The formats include:

      NumPy array 3x1   "p"     Point vector
      NumPy array 3x1   "e"     Axis (unit) vector
      NumPy array 3x1   "e"     Error vector
      NumPy array 3x3   "R"     Rotation matrix
      NumPy array 1x4   "quat"  Quaternion
      NumPy array 4x4   "T"     Transform matrix

   as well as the python list URDF tags <origin> and <axis>:

      Python List 1x3:  <axis>          Axis vector
      Python List 1x6:  <origin>        6D Transform Information
      Python List 1x3:  <origin> "xyz"  Vector of Positions
      Python List 1x3:  <origin> "rpy"  Vector of Euler Angles

   The Helper functions are:

   Cross Product:   cross(e1,e2)    Cross product of two 3x1 vectors
                    crossmat(e)     Cross product matrix

   Position         pzero()         Zero position vector
                    pxyz(x,y,z)     Position vector

   Axis Vectors     ex()            Unit x-axis
                    ey()            Unit y-axis
                    ez()            Unit z-axis
                    exyz(x,y,z)     Unit vector

   Rotation Matrix  Reye()          Identity rotation matrix
                    Rotx(alpha)     Rotation matrix about x-axis
                    Roty(alpha)     Rotation matrix about y-axis
                    Rotz(alpha)     Rotation matrix about z-axis
                    Rote(e, alpha)  Rotation matrix about unit vector e

   Error Vectors    ep(pd, p)       Translational error vector
                    eR(Rd, R)       Rotational error vector

   Transforms       T_from_Rp(R,p)  Compose T matrix
                    p_from_T(T)     Extract position vector from T
                    R_from_T(T)     Extract rotation matrix from T

   Quaternions      R_from_quat(quat)   Convert quaternion to R
                    quat_from_R(R)      Convert R to quaternion

   URDF Elements    T_from_URDF_origin(origin)   Construct transform
                    e_from_URDF_axis(axis)       Construct axis vector

'''

import numpy as np

from urdf_parser_py.urdf import Robot


#
#   Cross Product
#
def cross(a,b):
    return crossmat(a) @ b

def crossmat(e):
    e = e.flatten()
    return np.array([[  0.0, -e[2],  e[1]],
                     [ e[2],   0.0, -e[0]],
                     [-e[1],  e[0],  0.0]])


#
#   3x1 Position Vector
#
def pzero():
    return np.zeros((3,1))
    
def pxyz(x,y,z):
    return np.array([[x],[y],[z]])


#
#   3x1 Axis (Unit Vector)
#
def ex():
    return exyz(1.0, 0.0, 0.0)
def ey():
    return exyz(0.0, 1.0, 0.0)
def ez():
    return exyz(0.0, 0.0, 1.0)
    
def exyz(x,y,z):
    return np.array([[x],[y],[z]]) / np.sqrt(x*x+y*y+z*z)


#
#   3x3 Rotation Matrix
#
def Reye():
    return np.eye(3)

def Rotx(alpha):
    return np.array([[1.0, 0.0          ,  0.0          ],
                     [0.0, np.cos(alpha), -np.sin(alpha)],
                     [0.0, np.sin(alpha),  np.cos(alpha)]])

def Roty(alpha):
    return np.array([[ np.cos(alpha), 0.0, np.sin(alpha)],
                     [ 0.0          , 1.0, 0.0          ],
                     [-np.sin(alpha), 0.0, np.cos(alpha)]])

def Rotz(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha), 0.0],
                     [np.sin(alpha),  np.cos(alpha), 0.0],
                     [0.0          ,  0.0          , 1.0]])

def Rote(e, alpha):
    ex = crossmat(e)
    return np.eye(3) + np.sin(alpha) * ex + (1.0-np.cos(alpha)) * ex @ ex


#
#   3x1 Error Vectors
#
def ep(pd, p):
    return (pd - p)

def eR(Rd, R):
    return 0.5 * (cross(R[0:3,0:1], Rd[0:3,0:1]) +
                  cross(R[0:3,1:2], Rd[0:3,1:2]) +
                  cross(R[0:3,2:3], Rd[0:3,2:3]))


#
#   4x4 Transform Matrix
#
#   Build the T matrix from R/P, or extract the R/p pieces.
#
def T_from_Rp(R, p):
    return np.vstack((np.hstack((R,p)),
                      np.array([0.0, 0.0, 0.0, 1.0])))

def p_from_T(T):
    return T[0:3,3:4]
def R_from_T(T):
    return T[0:3,0:3]


#
#   1x4 Quaternions
#
#   Convert to/from a rotation matrix.
#
def R_from_quat(quat):
    q     = quat.flatten()
    norm2 = np.inner(q,q)
    w     = q[0]
    v     = q[1:].reshape((3,1))
    R     = (2/norm2) * (v@v.T + w*w*Reye() + w*crossmat(v)) - Reye()
    return R

def quat_from_R(R):
    A = [1.0 + R[0][0] + R[1][1] + R[2][2],
         1.0 + R[0][0] - R[1][1] - R[2][2],
         1.0 - R[0][0] + R[1][1] - R[2][2],
         1.0 - R[0][0] - R[1][1] + R[2][2]]
    i = A.index(max(A))
    A = A[i]
    c = 0.5/np.sqrt(A)
    if   (i == 0):
        q = c*np.array([A, R[2][1]-R[1][2], R[0][2]-R[2][0], R[1][0]-R[0][1]])
    elif (i == 1):
        q = c*np.array([R[2][1]-R[1][2], A, R[1][0]+R[0][1], R[0][2]+R[2][0]])
    elif (i == 2):
        q = c*np.array([R[0][2]-R[2][0], R[1][0]+R[0][1], A, R[2][1]+R[1][2]])
    else:
        q = c*np.array([R[1][0]-R[0][1], R[0][2]+R[2][0], R[2][1]+R[1][2], A])
    return q


#
#   URDF <origin> element
#
#   The <origin> element should contain "xyz" and "rpy" attributes:
#     origin.xyz  x/y/z coordinates of the position
#     origin.rpy  Euler angles for roll/pitch/yaw or x/y/z rotations
#
def p_from_URDF_xyz(xyz):
    return np.array(xyz).reshape((3,1))

def R_from_URDF_rpy(rpy):
    return Rotz(rpy[2]) @ Roty(rpy[1]) @ Rotx(rpy[0])

def T_from_URDF_origin(origin):
    return T_from_Rp(R_from_URDF_rpy(origin.rpy), p_from_URDF_xyz(origin.xyz))


#
#   URDF <axis> element
#
#   The <axis> element should contain only an "xyz" attribute.  Being
#   a single attribute, the parser directly presents this:
#     axis        x/y/z coordinates of the axis
#
def e_from_URDF_axis(axis):
    return np.array(axis).reshape((3,1))



#
#   Main Test Code
#
#   Simply test the above functions to make sure we have them correct.
#
if __name__ == "__main__":
    # Prepare the print format.
    np.set_printoptions(precision=6, suppress=True)

    # Test...
    R = Rotx(np.radians(45))
    print("R:\n", R)

    quat = quat_from_R(R)
    print("quat:\n", quat)

    print("R_from_quat():\n",  R_from_quat(quat))
