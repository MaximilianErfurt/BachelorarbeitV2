import cv2
import scipy.linalg
from PyQt5.QtGui import QImage
import numpy as np
from scipy.spatial.transform import Rotation as R


def grey_to_QImage(image):
    # convert to rgb
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # make QImage from rgb image
    height, width, channels = rgb_image.shape
    bytes_per_line = channels * width
    qimage = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
    return qimage

def pose_to_transformation_matrix(pose):
    """
    Turns x,y,z,a,b,c into transformation matrix
    :param pose: x,y,z,a,b,c
    :return: transformation matrix
    """
    x = float(pose[0])
    y = float(pose[1])
    z = float(pose[2])
    a = float(pose[3])
    b = float(pose[4])
    c = float(pose[5])
    d11 = np.cos(a) * np.cos(b)
    d12 = np.cos(a) * np.sin(b) * np.sin(c) - np.sin(a) * np.cos(c)
    d13 = np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c)
    d21 = np.sin(a) * np.cos(b)
    d22 = np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c)
    d23 = np.sin(a) * np.sin(b) * np.cos(c) - np.cos(a) * np.sin(c)
    d31 = -np.sin(b)
    d32 = np.cos(b) * np.sin(c)
    d33 = np.cos(b) * np.cos(c)
    transformation_matrix = [[d11, d12, d13, x], [d21, d22, d23, y], [d31, d32, d33, z], [0, 0, 0, 1]]
    return transformation_matrix

def r_vec_to_r_mat(r_vec):
    r_vec = r_vec.flatten()
    rotation = R.from_rotvec(r_vec)
    r_mat = rotation.as_matrix()
    return r_mat


def rvec_to_rmat(rvec):
    """
    transforms rotational vector to rotational matrix
    :param rvec:
    :return:
    """
    # normalized rvec
    u = rvec / np.linalg.norm(rvec)
    # rotation angle
    theta = np.linalg.norm(rvec)
    # cross product matrix
    k = [[0.0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]]
    # rotation matrix
    r_mat = np.eye(3) + np.sin(theta) * k + (1 - np.cos(theta)) * np.dot(k, k)
    return r_mat

def r_mat_to_r_vec(r_mat):
    """
    transforms a rotational matrix into a rotational vector
    :param r_mat: rotational matrix
    :return:
    """
    rotation = R.from_matrix(r_mat)
    r_vec = rotation.as_rotvec()
    return r_vec

def t_in_R_and_v(t_mat):
    v_f_f = np.array(
        [
            [t_mat[0,3]],
            [t_mat[1,3]],
            [t_mat[2,3]]
        ]
    )
    f_R_f = np.array(
        [
            [t_mat[0,0], t_mat[0,1], t_mat[0,2]],
            [t_mat[1,0], t_mat[1,1], t_mat[1,2]],
            [t_mat[2,0], t_mat[2,1], t_mat[2,2]]
        ]
    )
    return f_R_f, v_f_f


def r_mat_and_t_vec_to_t_mat(r_mat, t_vec):
    """
    combine rotational matrix and translational vector to transformation matrix
    :param r_mat: rotational matrix
    :param t_vec: translational vector
    :return: transformation matrix
    """
    t_vec = np.reshape(t_vec, (3,))
    t_mat = np.eye(4)
    t_mat[:3, :3] = r_mat
    t_mat[:3, 3] = t_vec
    return t_mat

def calc_x_T_x(x_mats):
    """
    calculates from point x to x+1
    :param x_mats:
    :return:
    """
    x_T_x = [x_mats[0] * np.linalg.inv(x_mats[1]), x_mats[1] * np.linalg.inv(x_mats[2]), x_mats[2] * np.linalg.inv(x_mats[3])]
    return x_T_x

def calc_hand_eye_transformation(f_T_fs, c_T_cs):
    # separate orientation and position
    f_R_fs = []     # array of flansch to flansch rotational matrices
    c_R_cs = []     # array of camera to camera rotational matrices
    v_f_fs = []     # array of flansch to flansch position vectors
    v_c_cs = []     # array of camera to camera position vectors
    f_v_fs = []     # array of flansch to flansch rotational vectors
    c_v_cs = []     # array of camera to camera rotational vectors
    A = []           # matrix from flansch to flansch rotational vectors
    B = []           # matrix from camera to camera rotational vectors

    # separate flansch to flansch matrices
    for f_T_f in f_T_fs:
        f_R_f, v_f_f = t_in_R_and_v(f_T_f)
        f_R_fs.append(f_R_f)
        v_f_fs.append(v_f_f)

    # separate camera to camera matrices
    for c_T_c in c_T_cs:
        c_R_c, v_c_c = t_in_R_and_v(c_T_c)
        c_R_cs.append(c_R_c)
        v_c_cs.append(v_c_c)

    # calc flansch to flansch rotational vectors
    for f_R_f in f_R_fs:
        f_v_fs.append(r_mat_to_r_vec(f_R_f))

    # calc camera to camera rotational vectors
    for c_R_c in c_R_cs:
        c_v_cs.append(r_mat_to_r_vec(c_R_c))

    # calc rotational matrix camera to flansch
    A = np.array([f_v_fs[0], f_v_fs[1], f_v_fs[2]])
    B = np.array([c_v_cs[0], c_v_cs[1], c_v_cs[2]])
    T = B.T @ A
    U, s, Vh = scipy.linalg.svd(T)
    Up = U @ Vh
    P = Vh.T @ np.diag(s) @ Vh
    Usp, Dp , Vhp = scipy.linalg.svd(P)
    [row, col] = P.shape
    f = np.linalg.det(Up)
    if f < 0:
        X = -np.eye(row, col)
    else:
        X = np.eye(row, col)
    Dx = Vhp.T @ X @ Vhp @ Up.T
    print("Dx" , Dx)

    # calc rx
    E = np.eye(3)
    C = np.vstack([
        E-c_R_cs[0],
        E-c_R_cs[1],
        E-c_R_cs[2]
    ])
    d0 = v_f_fs[0].flatten() - Dx @ v_c_cs[0].flatten()
    d1 = v_f_fs[1].flatten() - Dx @ v_c_cs[1].flatten()
    d2 = v_f_fs[2].flatten() - Dx @ v_c_cs[2].flatten()
    d = np.concatenate(
        [
            d0, d1, d2
        ]
    )
    print("d", d)
    CC = C.T @ C
    print("cc", CC)
    rx = np.linalg.pinv(CC) @ C.T @ d
    print("rx", rx)
    c_T_f = np.array(
        [
            [Dx[0,0], Dx[0,1], Dx[0,2], rx[0]],
            [Dx[1,0], Dx[1,1], Dx[1,2], rx[1]],
            [Dx[2,0], Dx[2,1], Dx[2,2], rx[2]],
            [0, 0, 0, 1]
        ]
    )
    return c_T_f

if __name__ == "__main__":
        pass