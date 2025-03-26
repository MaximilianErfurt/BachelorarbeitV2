import cv2
import scipy.linalg
from PyQt5.QtGui import QImage
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import fractional_matrix_power
from sklearn.cluster import DBSCAN
import json
import os
import matplotlib.pyplot as plt



def image_to_QImage(image):
    # check if image is gray
    if len(image.shape)==2:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # make QImage from rgb image
    height, width, channels = image.shape
    bytes_per_line = channels * width
    qimage = QImage(image.data, width, height, bytes_per_line, QImage.Format_RGB888)
    return qimage

def pose_to_transformation_matrix(pose):
    """
    Turns x,y,z,a,b,c into transformation matrix
    :param pose: x,y,z,a,b,c
    :return: transformation matrix
    """
    # get poses
    x, y, z, rx, ry, rz = map(float, pose)

    # transform euler angles to rotation matrix
    #rotation = R.from_euler("zyx", [rx, ry, rz])
    #r_mat = rotation.as_matrix()
    r_mat, _ = cv2.Rodrigues(np.array([[rx], [ry], [rz]]))

    # create 4x4 transformationmatrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = r_mat
    transformation_matrix[:3, 3] = [x, y, z]

    return transformation_matrix

def r_vec_to_r_mat(r_vec):
    r_vec = r_vec.flatten()
    rotation = R.from_rotvec(rotvec=r_vec)
    r_mat = rotation.as_matrix()
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
    x_T_x = [x_mats[0] @ np.linalg.inv(x_mats[1]), x_mats[1] @ np.linalg.inv(x_mats[2]), x_mats[2] @ np.linalg.inv(x_mats[3])]
    return x_T_x

# def calc_hand_eye_transformation(f_T_fs, c_T_cs):
#     # separate orientation and position
#     f_R_fs = []     # array of flansch to flansch rotational matrices
#     c_R_cs = []     # array of camera to camera rotational matrices
#     v_f_fs = []     # array of flansch to flansch position vectors
#     v_c_cs = []     # array of camera to camera position vectors
#     f_v_fs = []     # array of flansch to flansch rotational vectors
#     c_v_cs = []     # array of camera to camera rotational vectors
#     A = []           # matrix from flansch to flansch rotational vectors
#     B = []           # matrix from camera to camera rotational vectors
#
#     # separate flansch to flansch matrices
#     for f_T_f in f_T_fs:
#         f_R_f, v_f_f = t_in_R_and_v(f_T_f)
#         f_R_fs.append(f_R_f)
#         v_f_fs.append(v_f_f)
#
#     # separate camera to camera matrices
#     for c_T_c in c_T_cs:
#         c_R_c, v_c_c = t_in_R_and_v(c_T_c)
#         c_R_cs.append(c_R_c)
#         v_c_cs.append(v_c_c)
#
#     # calc flansch to flansch rotational vectors
#     for f_R_f in f_R_fs:
#         f_v_fs.append(r_mat_to_r_vec(f_R_f))
#
#     # calc camera to camera rotational vectors
#     for c_R_c in c_R_cs:
#         c_v_cs.append(r_mat_to_r_vec(c_R_c))
#
#     # calc rotational matrix camera to flansch
#     A = np.array([f_v_fs[0], f_v_fs[1], f_v_fs[2]])
#     B = np.array([c_v_cs[0], c_v_cs[1], c_v_cs[2]])
#     T = B.T @ A
#     U, s, Vh = scipy.linalg.svd(T)
#     Up = U @ Vh
#     P = Vh.T @ np.diag(s) @ Vh
#     Usp, Dp , Vhp = scipy.linalg.svd(P)
#     [row, col] = P.shape
#     f = np.linalg.det(Up)
#     if f < 0:
#         X = -np.eye(row, col)
#     else:
#         X = np.eye(row, col)
#     Dx = Vhp.T @ X @ Vhp @ Up.T
#     print("Dx" , Dx)
#
#     # calc rx
#     E = np.eye(3)
#     C = np.vstack([
#         E-c_R_cs[0],
#         E-c_R_cs[1],
#         E-c_R_cs[2]
#     ])
#     d0 = v_f_fs[0].flatten() - Dx @ v_c_cs[0].flatten()
#     d1 = v_f_fs[1].flatten() - Dx @ v_c_cs[1].flatten()
#     d2 = v_f_fs[2].flatten() - Dx @ v_c_cs[2].flatten()
#     d = np.concatenate(
#         [
#             d0, d1, d2
#         ]
#     )
#     print("d", d)
#     CC = C.T @ C
#     print("cc", CC)
#     rx = np.linalg.pinv(CC) @ C.T @ d
#     print("rx", rx)
#     c_T_f = np.array(
#         [
#             [Dx[0,0], Dx[0,1], Dx[0,2], rx[0]],
#             [Dx[1,0], Dx[1,1], Dx[1,2], rx[1]],
#             [Dx[2,0], Dx[2,1], Dx[2,2], rx[2]],
#             [0, 0, 0, 1]
#         ]
#     )
#     print("c_T_f",c_T_f)
#     return c_T_f

def grey_to_rgb(image):
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

def dbscan_clustering(image):
    # extract white pixels
    coords = np.column_stack(np.where(image>127))

    # execute DBSCAN
    dbscan = DBSCAN(eps = 7, min_samples=20)
    labels = dbscan.fit_predict(coords)

    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    print(f"Anzahl der Cluster: {n_clusters}")

    unique_labels = set(labels)

    centers = []
    for label in unique_labels:
        if label == -1:
            # ignore noise
            continue
        label_coords = coords[labels == label]
        size = len(label_coords)
        x_mean = np.mean(label_coords[:, 0])
        y_mean = np.mean(label_coords[:, 1])
        centers.append((x_mean, y_mean, size))
        print (x_mean, y_mean, size)
    return centers

def load_rob_poses(next_counter):
    path = "C:/Users/Maximilian Erfurt/PycharmProjects/BachelorarbeitV2/rob_poses.json"
    #path = "C:/Users/Stevi/PycharmProjects/BachelorarbeitV2/rob_poses.json"
    with open(path, "r") as file:
        data = json.load(file)
    poses = data["Posen"]
    pose = poses["p" + str(next_counter)]
    x = pose["x"]
    y = pose["y"]
    z = pose["z"]
    rx = pose["rx"]
    ry = pose["ry"]
    rz = pose["rz"]
    return x,y,z,rx,ry,rz


def load_image(next_counter):
    #path = "C:/Users/Stevi/Desktop/Bachelorarbeit/Quellen/Camera_Calibration/Camera_Calibration/images"
    #file_name = f"p{next_counter + 1:02d}.png"
    #path = "C:/Users/Stevi/PycharmProjects/BachelorarbeitV2/images/"
    path = "C:/Users/Maximilian Erfurt/PycharmProjects/BachelorarbeitV2/images/"
    file_name = str(next_counter) + ".png"
    image_path = os.path.join(path, file_name)
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    return image#

def save_image(next_counter, image):
    path = "C:/Users/Maximilian Erfurt/PycharmProjects/BachelorarbeitV2/images"
    # os.makedirs(path, exist_ok=True)
    file_name = os.path.join(path, f"{next_counter}.png")

    success = cv2.imwrite(file_name, image)
    if success:
            print(f"Image saved: {file_name}")
    else:
        print("error while saveing image!")


#-----------------------------------------------------------------
# functions for hand eye calibration
#-----------------------------------------------------------------
def r_and_t_to_T(r_vec, t_vec):
    """

    :param r_vec:
    :param t_vec:
    :return:
    """
    c_T_cb = np.eye(4, 4)
    c_T_cb[0:3, 3] = (np.transpose(t_vec))
    cv2.Rodrigues(r_vec, c_T_cb[0:3, 0:3])
    return c_T_cb

def solve_ax_xb(A_i, B_i):
    # tA = np.array([A[0:3, 3] for A in A_i])  # Flanschpositionen
    # tB = np.array([B[0:3, 3] for B in B_i])  # Markerpositionen
    #
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    #
    # for i, (x, y, z) in enumerate(tA):
    #     ax.scatter(x, y, z, color="red")
    #     ax.text(x, y, z, str(i), color="red")
    #
    # for i, (x, y, z) in enumerate(tB):
    #     ax.scatter(x, y, z, color="blue")
    #     ax.text(x, y, z, str(i), color="blue")
    #
    # ax.legend(["Flanschpositionen", "Markerpositionen"])
    # plt.show()

    rA_i = []
    rB_i = []
    tA_i = []
    tB_i = []
    DA_i = []
    DB_i = []
    for i in range(len(A_i)):
        DA = A_i[i][0:3, 0:3]       # separate rotational part of A_i
        DA_i.append(DA)
        DA = R.from_matrix(matrix=DA)   # convert rot_mat into rot_vec
        A_rvec = DA.as_rotvec()

        DB = B_i[i][0:3, 0:3]       # separate rotational part of B_i
        DB_i.append(DB)
        DB = R.from_matrix(matrix=DB)   # convert rot_mat into rot_vec
        B_rvec = DB.as_rotvec()

        if i == 0:      # add all rot_vecs from A and B into one matrix rA_i, rB_i
            rA_i = A_rvec
            rB_i = B_rvec
        else:
            rA_i = np.c_[rA_i, A_rvec]
            rB_i = np.c_[rB_i, B_rvec]

        tA_i.append(A_i[i][0:3, 3])     # separate translation part of A_i
        tB_i.append(B_i[i][0:3, 3])     # separate translation part of B_i

    T = np.dot(rB_i, np.transpose(rA_i))

    # svd
    U, s, Vh = scipy.linalg.svd(T)
    Up = np.dot(U, Vh)
    P = np.linalg.multi_dot([np.transpose(Vh), np.diag(s), Vh])
    Usp, Dp, Vhp = scipy.linalg.svd(P)



    row, col = np.shape(P)
    f = np.linalg.det(Up)
    if f<0:
        X = np.eye(row, col) * (-1)
    else:
        X = np.eye(row, col)

    # calculate Rx
    Rx = np.linalg.multi_dot([np.transpose(Vhp), X, Vhp, np.transpose(Up)])
    u = fractional_matrix_power(np.dot(np.transpose(T), T), -0.5)
    Xx = np.dot(fractional_matrix_power(np.dot(np.transpose(T), T), t = -0.5), np.transpose(T))
    print("Rx: \n", Rx)
    print("Xx: \n", Xx)
    # calculate tx
    E = np.eye(3, 3)
    Ci = None
    di = None
    for i in range(len(A_i)):
        C = np.subtract(E,A_i[i][0:3, 0:3])
        d = np.subtract(tA_i[i], np.dot(Rx, tB_i[i]))

        if i == 0:
            Ci = C
            di = np.array(np.transpose([d]))
        else:
            Ci = np.vstack((Ci, C))
            di = np.vstack((di, np.array(np.transpose([d]))))

    print("Bedingungszahl von C_i:", np.linalg.cond(Ci))
    tx = np.linalg.multi_dot([np.linalg.pinv(np.dot(np.transpose(Ci), Ci)), np.transpose(Ci), di])
    # tx = np.dot(np.linalg.pinv(Ci), di)
    print("tx: \n", tx)

    X = np.eye(4, 4)
    X[0:3, 0:3] = Rx
    X[0:3, 3] = np.transpose(tx)

    return X, Rx, tx, DA_i, DB_i, tA_i, tB_i
















