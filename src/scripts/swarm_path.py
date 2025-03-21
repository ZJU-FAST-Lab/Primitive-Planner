"""
retime a path for uav
=======================================
Dof: 2 [x, y]; z = 0
"""

from os import mkdir
from matplotlib import projections
from pyrfc3339 import generate
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from scipy.interpolate import CubicSpline

import scipy.spatial as spatial
from scipy.spatial.transform import Rotation as R


import struct

# from torch import int16, short
import math
import shutil
import os

ta.setup_logging("INFO")

global max_vel

def delete_directory_contents(dir_path):
    """
    Delete all files and folders in the specified directory.

    :param dir_path: The directory path to be cleaned up
    """
    # Check if the directory exists
    if not os.path.exists(dir_path):
        print(f"The directory {dir_path} does not exist.")
        return

    # Iterate over all files and folders in the directory
    for filename in os.listdir(dir_path):
        file_path = os.path.join(dir_path, filename)
        
        # If it is a file or a link, delete it
        if os.path.isfile(file_path) or os.path.islink(file_path):
            os.unlink(file_path)
            print(f"Deleted file: {file_path}")
        
        # If it is a directory, recursively delete it
        elif os.path.isdir(file_path):
            shutil.rmtree(file_path)
            print(f"Deleted directory: {file_path}")

    print("Cleanup complete.")
    time.sleep(1)

def radius_theta(arc_length, arc_radius):
    # set primitive len = pi * 6.0 / 2 = 3 * pi(1/4 circle r = 6)
    # (0.5 * pi - scale * pi ) * r = len
    scale = 0.5 - arc_length / (np.pi * arc_radius)
    return scale

# frame x-axis forward  
def generate_path(max_vel, max_acc, arc_length, arc_radius):
    N_sample = 5

    if math.isinf(arc_radius):
        # line
        ss = np.linspace(0, 1, N_sample)
        way_pts = np.linspace(0, arc_length, N_sample).reshape(N_sample, 1)
        vlims = np.array([max_vel])
        alims = np.array([max_acc])
    else:
        ss = np.linspace(0, 1, N_sample)
        theta_end = radius_theta(arc_length, arc_radius) if radius_theta(arc_length, arc_radius) > 0 else 0
        theta = np.linspace(0.5 * np.pi, theta_end * np.pi, N_sample)
        way_pts = np.c_[arc_radius * (np.cos(theta)), arc_radius *(np.sin(theta) - 1)]
        vlims = np.array([max_vel, max_vel])
        alims = np.array([max_acc, max_acc])

    return ss, way_pts, vlims, alims

def inv_dyn(q, qs, qss):
    qst = qs.T
    w = np.array([qst.dot(qs)])
    return w

def constraintF(dim):
    F = np.array([[1.0]])
    return F

def constraintg(dim):
    global max_vel
    # 9.1 - 3^2(vlim_norm = 3.0); 9.1 to make start_vel = 3.0 feasible (machine precision)
    g = np.array([max_vel ** 2 + 0.01])
    return g

def solveHorizonToppAndSample(radius, ss, way_pts, vlims, alims, v_start, v_end):
    path = ta.SplineInterpolator(ss, way_pts)
    pc_vel = constraint.JointVelocityConstraint(vlims)
    pc_acc = constraint.JointAccelerationConstraint(alims,discretization_scheme=constraint.DiscretizationType.Interpolation)
    gridpoints = np.linspace(0, path.duration, 1000)  # 1000 points

    if math.isinf(radius):
        instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, parametrizer="ParametrizeSpline")
        s_start = v_start / path(gridpoints[0], 1)
        s_end = v_end / path(gridpoints[-1], 1)
    else:
        pc_vel_norm = constraint.SecondOrderConstraint(inv_dyn, constraintF, constraintg, dof=2)
        instance = algo.TOPPRA([pc_vel, pc_acc, pc_vel_norm], path, gridpoints=gridpoints, parametrizer="ParametrizeSpline")
        # print("a %.3f", path(gridpoints[0], 1))
        # print("b %.3f", path(gridpoints[-1], 1))
        s_start = v_start / path(gridpoints[0], 1)[0]
        s_end = v_end / abs(path(gridpoints[-1], 1)[1])


    jnt_traj = instance.compute_trajectory(s_start, s_end)
    if jnt_traj == None:
        raise Exception('radius={} and v_start={} solve TOPP infeasible!!!'.format(radius, v_start))

    # sample 0.01s resolution -> traj_server 100Hz
    ts_sample = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration * 100))
    traj_duration = jnt_traj.duration

    if math.isinf(radius):
        pos_x = jnt_traj(ts_sample)
        vel_x = jnt_traj(ts_sample, 1)
        acc_x = jnt_traj(ts_sample, 2)
        pos_yz = np.zeros((pos_x.shape[0], 2))
        vel_yz = np.zeros((vel_x.shape[0], 2))
        acc_yz = np.zeros((acc_x.shape[0], 2))
        pos_xyz = np.c_[pos_x, pos_yz]
        vel_xyz = np.c_[vel_x, vel_yz]
        acc_xyz = np.c_[acc_x, acc_yz]

        # plot 
        # fig, axs = plt.subplots(3, 1, sharex=True)
        # axs[0].plot(ts_sample, pos_x)
        # axs[1].plot(ts_sample, vel_x)
        # axs[2].plot(ts_sample, acc_x)
        # plt.show()
    else:
        pos_xy = jnt_traj(ts_sample)
        vel_xy = jnt_traj(ts_sample, 1)
        acc_xy = jnt_traj(ts_sample, 2)
        pos_z = np.zeros(pos_xy.shape[0])
        vel_z = np.zeros(vel_xy.shape[0])
        acc_z = np.zeros(acc_xy.shape[0])
        pos_xyz = np.c_[pos_xy, pos_z]
        vel_xyz = np.c_[vel_xy, vel_z]
        acc_xyz = np.c_[acc_xy, acc_z]

        # plot
        # fig, axs = plt.subplots(3, 1, sharex=True)
        # for i in range(path.dof):
        #     # plot the i-th joint trajectory
        #     axs[0].plot(ts_sample, pos_xy[:, i], c="C{:d}".format(i))
        #     axs[1].plot(ts_sample, vel_xy[:, i], c="C{:d}".format(i))
        #     axs[2].plot(ts_sample, acc_xy[:, i], c="C{:d}".format(i))
        # axs[2].set_xlabel("Time (s)")
        # axs[0].set_ylabel("Position")
        # axs[1].set_ylabel("Velocity")
        # axs[2].set_ylabel("Acceleration")
        # plt.show()

    return pos_xyz, vel_xyz, acc_xyz, traj_duration

def fileVelPathID(lib_dir, v_start, path_id, pos_xyz, vel_xyz, acc_xyz, traj_duration):
    filename = lib_dir + f"trajectory/{int(round(v_start*10))}/{path_id}_trajectory.ply"
    file_trajectory = open(filename, 'w')
    traj_combine = np.c_[pos_xyz, vel_xyz, acc_xyz]
    print('%d'%traj_combine.shape[0], file=file_trajectory)
    print('%f'%traj_duration, file=file_trajectory)

    for i in range(traj_combine.shape[0]):
        print('%f'%traj_combine[i,0], '%f'%traj_combine[i,1], '%f'%traj_combine[i,2], '%f'%traj_combine[i,3], '%f'%traj_combine[i,4], '%f'%traj_combine[i,5], '%f'%traj_combine[i,6], '%f'%traj_combine[i,7],'%f'%traj_combine[i,8], file=file_trajectory)
    file_trajectory.close()

def filePosPathID(lib_dir, v_start, path_id, pos_xyz, traj_duration):
    filename = lib_dir + f"trajectory_pos/{int(round(v_start*10))}/{path_id}_trajectory.ply"
    file_trajectory = open(filename, 'w')
    print('%d'%pos_xyz.shape[0], file=file_trajectory)
    print('%f'%traj_duration, file=file_trajectory)

    for i in range(pos_xyz.shape[0]):
        print('%f'%pos_xyz[i,0], '%f'%pos_xyz[i,1], '%f'%pos_xyz[i,2], file=file_trajectory)
    file_trajectory.close()

def filePathAll(lib_dir, pos_all):
    file_path_all = open(lib_dir + 'obs_correspondence/path_all.ply', 'w')
    # print('ply', file=file_path_all)
    # print('format ascii 1.0', file=file_path_all)
    # print('element vertex %d'%pos_all.shape[0], file=file_path_all)
    # print('property float x', file=file_path_all)
    # print('property float y', file=file_path_all)
    # print('property float z', file=file_path_all)
    # print('property int path_id', file=file_path_all)
    # print('end_header', file=file_path_all)
    print('%d'%pos_all.shape[0], file=file_path_all)
    for i in range(pos_all.shape[0]):
        print('%f'%pos_all[i,0], '%f'%pos_all[i,1], '%f'%pos_all[i,2], '%d'%pos_all[i,3], file=file_path_all)
    file_path_all.close()

def filePathEnd(lib_dir, primitive_end_all):
    file_path_end = open(lib_dir + 'obs_correspondence/path_end.ply', 'w')
    # print('ply', file=file_path_end)
    # print('format ascii 1.0', file=file_path_end)
    # print('element vertex %d'%primitive_end_all.shape[0], file=file_path_end)
    # print('property float x', file=file_path_end)
    # print('property float y', file=file_path_end)
    # print('property float z', file=file_path_end)
    # print('property int path_id', file=file_path_end)
    # print('end_header', file=file_path_end)
    for i in range(primitive_end_all.shape[0]):
        print('%f'%primitive_end_all[i,0], '%f'%primitive_end_all[i,1], '%f'%primitive_end_all[i,2], '%d'%primitive_end_all[i,3], file=file_path_end)
    file_path_end.close()

def obsCorrespondence(lib_dir, box_x, box_y, box_z, pos_all, obs_inflate_radius, voxelSize):
    voxel_x = box_x
    voxel_y = box_y / 2
    voxel_z = box_z / 2

    voxelNumX = int(box_x / voxelSize)
    voxelNumY = int(box_y / voxelSize)
    voxelNumZ = int(box_z / voxelSize)

    idxPoint = 0
    voxelPointNum = voxelNumX * voxelNumY * voxelNumZ
    voxelPoints = np.zeros(shape=(voxelPointNum, 3))
    for indX in range(voxelNumX):
        x = voxel_x - voxelSize * indX
        for indY in range(voxelNumY):
            y = voxel_y - voxelSize * indY
            for indZ in range(voxelNumZ):
                z = voxel_z - voxelSize * indZ

                voxelPoints[idxPoint, 0] = x - 0.5 * voxelSize
                voxelPoints[idxPoint, 1] = y - 0.5 * voxelSize
                voxelPoints[idxPoint, 2] = z - 0.5 * voxelSize
                idxPoint += 1

    # ax.scatter3D(voxelPoints[:, 0], voxelPoints[:, 1], voxelPoints[:, 2], c='k')
    # plt.show()

    # TODO:[resolution] kd-tree search radius
    # inflate map 0.1
    path_tree = spatial.cKDTree(pos_all[:, 0:3])
    ind = path_tree.query_ball_point(voxelPoints, obs_inflate_radius)
    file_correspondence = open(lib_dir + 'obs_correspondence/obs_correspondence.txt', 'wb+')
    for i in range(voxelPointNum):
        # struct.pack 转换为二进制数据
        file_correspondence.write(struct.pack("i", i))
        # ind[i]里面的值不可能重复，都是相互独立的
        indVoxel = sorted(ind[i])
        indVoxelNum = len(indVoxel)

        pathIndRec = int(-1)
        for j in range(indVoxelNum):
            pathInd = int(pos_all[indVoxel[j], 3])
            # 避免了pathInd重复
            if pathInd == pathIndRec:
                continue

            file_correspondence.write(struct.pack("i", pathInd))
            pathIndRec = pathInd
        
        file_correspondence.write(struct.pack("i", -1))
    file_correspondence.close()

def agentCorrespondence(lib_dir, box_x, box_y, box_z, pos_all, agent_inflate_radius, voxelSize, v_start):
    global script_dir
    voxel_x = box_x
    voxel_y = box_y / 2
    voxel_z = box_z / 2

    voxelNumX = int(box_x / voxelSize)
    voxelNumY = int(box_y / voxelSize)
    voxelNumZ = int(box_z / voxelSize)

    idxPoint = 0
    voxelPointNum = voxelNumX * voxelNumY * voxelNumZ
    voxelPoints = np.zeros(shape=(voxelPointNum, 3))
    for indX in range(voxelNumX):
        x = voxel_x - voxelSize * indX
        for indY in range(voxelNumY):
            y = voxel_y - voxelSize * indY
            for indZ in range(voxelNumZ):
                z = voxel_z - voxelSize * indZ

                voxelPoints[idxPoint, 0] = x - 0.5 * voxelSize
                voxelPoints[idxPoint, 1] = y - 0.5 * voxelSize
                voxelPoints[idxPoint, 2] = z - 0.5 * voxelSize
                idxPoint += 1

    # ax.scatter3D(voxelPoints[:, 0], voxelPoints[:, 1], voxelPoints[:, 2], c='k')
    # plt.show()

    # TODO:[resolution] kd-tree search radius
    path_tree = spatial.cKDTree(pos_all[:, 0:3])
    ind = path_tree.query_ball_point(voxelPoints, agent_inflate_radius)
    file_correspondence = open(lib_dir + f'agent_correspondence/{int(round(v_start*10))}_correspondence.txt', 'wb+')
    for i in range(voxelPointNum):
        # struct.pack 转换为二进制数据
        file_correspondence.write(struct.pack("i", i))
        # ind[i]里面的值不可能重复，都是相互独立的
        indVoxel = sorted(ind[i])
        indVoxelNum = len(indVoxel)

        pathIndRec = int(-1)
        tCur = 0
        tLast = 0
        for j in range(indVoxelNum):
            pathInd = int(pos_all[indVoxel[j], 3])
            pathTime =  int(pos_all[indVoxel[j], 4])
            tCur = pathTime

            if pathInd != pathIndRec:
                if j == 0:
                    file_correspondence.write(struct.pack("i", pathInd))
                    file_correspondence.write(struct.pack("i", tCur))
                else:
                    file_correspondence.write(struct.pack("i", tLast))
                    file_correspondence.write(struct.pack("i", pathInd))
                    file_correspondence.write(struct.pack("i", tCur))
            
            if j == indVoxelNum - 1:
                file_correspondence.write(struct.pack("i", tCur))

            tLast = pathTime
            pathIndRec = pathInd

        file_correspondence.write(struct.pack("i", -1))
    file_correspondence.close()

def timeSpaceCorrespondence(lib_dir, max_vel, max_acc, arc_length, arc_radius, delta_angle, drone_radius, voxelSize, box_x, box_y, box_z, plot):
    v_start = 0.0
    v_end = 0.0
    mkdir(lib_dir + f"trajectory/")
    mkdir(lib_dir + f"trajectory_pos/")
    mkdir(lib_dir + f"agent_correspondence/")
    mkdir(lib_dir + f"obs_correspondence/")
    while v_start < max_vel + 0.05:
        # generate pos_all
        mkdir(lib_dir + f"trajectory/{int(round(v_start*10))}/")
        mkdir(lib_dir + f"trajectory_pos/{int(round(v_start*10))}/")
        filename = lib_dir + f"trajectory/{int(round(v_start*10))}/{int(round(v_start*10))}_infeasible_id.ply"
        file_infeasible_id = open(filename, 'w')

        path_id = -1
        # [x y z path_id time]
        pos_all = np.zeros(shape=(0, 5))
        # reserve vel = max_vel --- pos_all and primitive_end_all
        # if abs(v_start - max_vel) < 1e-3:
        # [x y z path_id]
        primitive_end_all = np.zeros(shape=(0, 4))
        
        for path_type in range(0, len(arc_radius)):
            ss, way_pts, vlims, alims = generate_path(max_vel, max_acc, arc_length, arc_radius[path_type])
            try:
                pos_xyz, vel_xyz, acc_xyz, traj_duration = solveHorizonToppAndSample(arc_radius[path_type], ss, way_pts, vlims, alims, v_start, v_end)
            except Exception as e:
                # vel -> infeasible_path_id.ply
                print("\033[33m" + str(e) + "\033[0m")
                if math.isinf(arc_radius[path_type]):
                    path_id += 1
                    print(path_id, file=file_infeasible_id)
                else:
                    for i in range(0, int(360 / delta_angle)):
                        path_id += 1
                        print(path_id, file=file_infeasible_id)
                continue

            # rotation
            if math.isinf(arc_radius[path_type]):
                fileVelPathID(lib_dir, v_start, path_id, pos_xyz, vel_xyz, acc_xyz, traj_duration)
                filePosPathID(lib_dir, v_start, path_id, pos_xyz, traj_duration)

                # add path_id time
                pos_add_id_time = np.c_[pos_xyz, path_id * np.ones(pos_xyz.shape[0], dtype=int), np.arange(0, pos_xyz.shape[0] * 10, 10)]
                pos_all = np.r_[pos_all, pos_add_id_time]

                # if abs(v_start - max_vel) < 1e-3:
                primitive_end_add_id = np.c_[pos_xyz[-1, :].reshape(1,3), path_id]
                primitive_end_all = np.r_[primitive_end_all, primitive_end_add_id]

                path_id += 1
            else:
                # construct delta angle [0 360, -10 350, -20 340, 0 360, ...]
                sub_angle = (path_type - 1) % 3
                for theta in range(- sub_angle * 10, 360 - sub_angle * 10, delta_angle):
                    r = R.from_euler('x', theta, degrees=True)
                    rot_mat = r.as_matrix()
                    rot_pos_xyz = np.dot(rot_mat, pos_xyz.T).T
                    rot_vel_xyz = np.dot(rot_mat, vel_xyz.T).T
                    rot_acc_xyz = np.dot(rot_mat, acc_xyz.T).T
                    fileVelPathID(lib_dir, v_start, path_id, rot_pos_xyz, rot_vel_xyz, rot_acc_xyz, traj_duration)
                    filePosPathID(lib_dir, v_start, path_id, rot_pos_xyz, traj_duration)
                    # add path_id time
                    pos_add_id_time = np.c_[rot_pos_xyz, path_id * np.ones(rot_pos_xyz.shape[0], dtype=int), np.arange(0, rot_pos_xyz.shape[0] * 10, 10)]
                    # print("pos_add_id_time=", pos_add_id_time)
                    pos_all = np.r_[pos_all, pos_add_id_time]

                    # if abs(v_start - max_vel) < 1e-3:
                    primitive_end_add_id = np.c_[rot_pos_xyz[-1, :].reshape(1,3), path_id]
                    primitive_end_all = np.r_[primitive_end_all, primitive_end_add_id]

                    path_id += 1

        # generate collision correspondence using agent and obstacles
        if v_start == 0.0:
            filePathAll(lib_dir, pos_all)
            filePathEnd(lib_dir, primitive_end_all)

            # visualize
            if plot:
                path_fig = plt.figure()
                ax = plt.axes(projection="3d")
                ax.scatter3D(pos_all[:, 0], pos_all[:, 1], pos_all[:, 2], c='g', s=5)
                ax.scatter3D(primitive_end_all[:, 0], primitive_end_all[:, 1], primitive_end_all[:, 2], c='r', s=5)
                max_val = max(abs(np.max(pos_all[:, 0]) / 2), abs(np.max(pos_all[:, 1])), abs(np.min(pos_all[:, 1])), abs(np.max(pos_all[:, 2])), abs(np.min(pos_all[:, 2])))
                ax.set_xlim([0, max_val * 2])
                ax.set_ylim([-max_val, max_val])
                ax.set_zlim([-max_val, max_val])
                # plt.show()

            obs_inflate_radius = math.sqrt(3) * voxelSize / 2.0 + drone_radius
            obsCorrespondence(lib_dir, box_x, box_y, box_z, pos_all, obs_inflate_radius, voxelSize)
            

        # generate collision correspondence using agent and agent
        agent_inflate_radius = math.sqrt(3) * voxelSize / 2.0 + 2 * drone_radius
        # agent_inflate_radius = 2 * drone_radius
        agentCorrespondence(lib_dir, box_x, box_y, box_z, pos_all, agent_inflate_radius, voxelSize, v_start)

        file_infeasible_id.close()
        v_start += 0.1
        
    # visualize
    if plot:
        plt.show()
            
def run(lib_dir, delta_angle=30, plot=False):
    global script_dir
    global max_vel
    max_vel = 1.0
    max_acc = 6.0
    arc_length = 5.0
    # arc radius set
    arc_radius = [float("inf"), 78.0, 36.0, 20.0, 12.0, 8.0, 6.0, 4.0, 3.0, 2.0, 1.5, 1.0, 0.5, 0.25, 0.125, 0.0625]
    # rotation delta_angle
    delta_angle = round(delta_angle)
    # drone radius
    drone_radius = 0.15
    # virtual grid resolution
    voxelSize = 0.1
    # box size coveraging all paths
    box_x = 6.0
    box_y = 6.0
    box_z = 6.0

    delete_directory_contents(lib_dir)
    time.sleep(1.0)
    os.makedirs(lib_dir, exist_ok=True)
    timeSpaceCorrespondence(lib_dir, max_vel, max_acc, arc_length, arc_radius, delta_angle, drone_radius, voxelSize, box_x, box_y, box_z, plot)


if __name__ == "__main__":
        
    script_dir = os.path.dirname(os.path.abspath(__file__))
    lib_dir = script_dir + '/../planner/plan_manage/primitive_library/'
    
    run(lib_dir, 30, True)
