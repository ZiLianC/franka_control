import json
import requests
import numpy as np
# from inference import inference
from gsnet_inference import inference
from transforms3d.quaternions import quat2mat, mat2quat
from metric.eval_grasp import GraspGroup
import open3d as o3d
import time
import copy
import os
# 172.19.53.58"
REALSENSE = {
    "address": "172.19.72.60",
    "port": 8000,
    "width": 1280,
    "height": 720,
    "depth_dtype": np.uint16,
    "color_dtype": np.uint8,
    "intrinsic_dtype": np.float64,
}

PANDA = {
    "address": "172.19.72.60",
    "port": 8001,
}


def get_depth():
    res = requests.get(f"http://{REALSENSE['address']}:{REALSENSE['port']}/depth")
    if res.status_code != 200:
        raise Exception("Realsense error!")
    depth_image = np.frombuffer(res.content, dtype=REALSENSE["depth_dtype"])
    depth_image = depth_image.reshape(REALSENSE["height"], REALSENSE["width"])
    return depth_image


def get_color():
    res = requests.get(f"http://{REALSENSE['address']}:{REALSENSE['port']}/color")
    if res.status_code != 200:
        raise Exception("Realsense error!")
    color_image = np.frombuffer(res.content, dtype=REALSENSE["color_dtype"])
    color_image = color_image.reshape(REALSENSE["height"], REALSENSE["width"], 3)
    return color_image    


def get_intrinsic():
    res = requests.get(
        f"http://{REALSENSE['address']}:{REALSENSE['port']}/intrinsic/color"
    )
    if res.status_code != 200:
        raise Exception("Realsense error!")
    color_intr = np.frombuffer(res.content, dtype=REALSENSE["intrinsic_dtype"])
    color_intr = color_intr.reshape(3, 3)
    return color_intr


def send_pose(pose):
    pose_data = {
        "tx": pose[0],
        "ty": pose[1],
        "tz": pose[2],
        "qw": pose[3],
        "qx": pose[4],
        "qy": pose[5],
        "qz": pose[6],
    }
    res = requests.post(f'http://{PANDA["address"]}:{PANDA["port"]}/pose', pose_data)
    if res.status_code != 200:
        raise Exception(res.text)


def get_pose():
    res = requests.get(f'http://{PANDA["address"]}:{PANDA["port"]}/pose')
    pose = json.loads(res.content)
    return [pose["tx"], pose["ty"], pose["tz"], pose["qw"], pose["qx"], pose["qy"], pose["qz"]]


def send_joint_angles(angles):
    joint_angles = {
        'joint1': angles[0],
        'joint2': angles[1],
        'joint3': angles[2],
        'joint4': angles[3],
        'joint5': angles[4],
        'joint6': angles[5],
        'joint7': angles[6],
    }
    res = requests.post(f'http://{PANDA["address"]}:{PANDA["port"]}/joint_angles', joint_angles)
    if res.status_code != 200:
        raise Exception(res.text)

def get_joint_angles():
    res = requests.get(f'http://{PANDA["address"]}:{PANDA["port"]}/joint_angles')
    angles = json.loads(res.content)
    return [angles["panda_joint1"], angles["panda_joint2"], angles["panda_joint3"], angles["panda_joint4"],
            angles["panda_joint5"], angles["panda_joint6"], angles["panda_joint7"]]

def open_finger():
    res = requests.get(f'http://{PANDA["address"]}:{PANDA["port"]}/open')
    pose = res.content.decode

def close_finger2():
    res = requests.get(f'http://{PANDA["address"]}:{PANDA["port"]}/close')
    pose = res.content.decode

def close_finger(width: float):
    res = requests.post(
        f'http://{PANDA["address"]}:{PANDA["port"]}/gripper', {"width": width}
    )
    if res.status_code != 200:
        raise Exception(res.text)

def further_decode(preds, cloud, rgb, RT_cam, gripper2base):
    rot_ext = RT_cam
    grasps = preds
    gg = GraspGroup(grasps)
    gg = gg.nms()
    grasps = gg.grasp_group_array
    grasps_mat = grasps[:, 4:13].reshape(-1, 3, 3)
    grasps_mat = np.matmul(rot_ext[:3, :3], grasps_mat)

    grasps_mat = np.matmul(gripper2base[:3, :3], grasps_mat)

    grasps_filter1 = grasps_mat[:, 2, 0] < -0.9
    grasps = grasps[grasps_filter1]
    print('grasps after nms: ', grasps.shape)

    indices = np.argsort(-grasps[:, 0])
    grasps = grasps[indices]
    # grasp_max_score_ind = 0 # (1,)
    # top_grasp = grasps[grasp_max_score_ind]  # (17,)
    # print('=> Top grasp in cam: ' + str(grasp.reshape(-1)))  # (17,)

    cloud = cloud.reshape(-1, 3)
    gg = GraspGroup(grasps[:20])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud)
    pcd.colors = o3d.utility.Vector3dVector(rgb/255.)
    grasps_geometry = gg.to_open3d_geometry_list()
    for i in range(len(grasps_geometry)):
        o3d.visualization.draw_geometries([pcd, grasps_geometry[i]])
        sel = input('select this pose: [Y/N]:')
        if sel == 'Y' or sel== 'y':
            top_grasp = grasps[i]
            break

    mat = top_grasp[4:13].reshape(3, 3)
    approach_vec = mat[:, 0]  # (3,)
    translation = top_grasp[13:16].reshape(3, 1)  # (3,1)
    print('=> Top grasp translation in cam: ' + str(translation.reshape(-1)))

    pose_in_cam = np.concatenate([mat, translation], axis=1)  # (3,4)
    pose_in_cam_cat = np.array([0., 0., 0., 1.]).reshape(1, 4)
    pose_in_cam = np.concatenate([pose_in_cam, pose_in_cam_cat], axis=0)  # (4,4)

    pose_in_base = np.dot(rot_ext, pose_in_cam)  # (4,4)
    pose_in_base = np.dot(gripper2base, pose_in_base)

    xyz = pose_in_base[:3, -1]  # (3,)
    pose_in_base_gripper_align = pose_in_base[:3, :3].copy()
    pose_in_base_gripper_align[:, 0] = pose_in_base[:3, 2]
    pose_in_base_gripper_align[:, 1] = -pose_in_base[:3, 1]
    pose_in_base_gripper_align[:, 2] = pose_in_base[:3, 0]
    if pose_in_base_gripper_align[0,0]<=0:
        pose_in_base_gripper_align = np.matmul(np.array([[-1,0,0],[0,-1,0],[0,0,1]]), pose_in_base_gripper_align)
        # return False
    quat = mat2quat(pose_in_base_gripper_align)  # (4)
    print('=> Grasp pose in robot base coordinate system: ')
    print('=> T: ' + str(xyz).strip())
    print('=> R: ' + str(quat).strip())

    final_gripper_grasp_pose = np.concatenate([xyz, quat])
    print('=> Pose: ' + str(final_gripper_grasp_pose).strip())
    print(final_gripper_grasp_pose)
    return final_gripper_grasp_pose

 
if __name__ == '__main__':
    import pdb
    pdb.set_trace()
    scene_id = 1
    init_position_joint_angles = [-0.08501774, -1.2, 0.08540554, -2.29133467, 0.12616069, 1.47892905, 0.80489255]
    end_position_joint_angles = [-1.139078255233016, -0.6600809494070021, 2.042231025160356, -1.5781708154063532, 
                                 0.6488107292743955, 1.8657994588216147, -0.0038481010713921912]
    send_joint_angles(init_position_joint_angles)
    time.sleep(1)
    # acquire data
    getdata = input("get data now: [Y/N] ")
    
    if getdata == 'y' or getdata == 'Y':
        depth = get_depth()
        color = get_color()
        camera = get_intrinsic()
        eepose = get_pose()
    else:
        raise ValueError('stop')
    
    gripper2base = np.eye(4)
    gripper_ori_mat = quat2mat([eepose[3], eepose[4], eepose[5], eepose[6]])
    gripper_tsf_vec = np.array(eepose[:3])
    gripper_tsf_vec = (gripper_tsf_vec - gripper_ori_mat[:, 2] * 0.10)
    gripper2base[:3,:3] = gripper_ori_mat
    gripper2base[:3,3] = gripper_tsf_vec
    
    # gg, refine_depth, refine_cloud, raw_cloud, rgb = inference(color, depth, camera)
    gg, raw_cloud, rgb = inference(color, depth, camera)
    # robot control
    quat_cam = np.array([0.700389, -0.000647178, -0.000132055, -0.713761])
    mat_cam = quat2mat(quat_cam)
    T_cam = np.array([0.0601015, -0.0127746, 0.0175023]).T.reshape(3,1)
    RT_cam = np.concatenate([mat_cam, T_cam], -1)
    RT_cam = np.concatenate([RT_cam, np.array([0, 0, 0, 1]).reshape(1, 4)])

    target_pose = further_decode(gg, raw_cloud, rgb, RT_cam, gripper2base)
    # send_joint_angles(init_position_joint_angles)
    pick = input("pick object now: [Y/N]")
    if pick == 'Y' or pick == 'y':
        init_pose = get_pose()
        print('start pre-grasp.............')
        bias = 0.3
        pre_grasp_pose = copy.deepcopy(target_pose)
        pre_grasp_pose[2] = pre_grasp_pose[2] + bias
        print('pre grasp pose:', pre_grasp_pose)
        send_pose(pre_grasp_pose)
        print('start grasp................')
        bias = -0.01
        target_pose[2] = target_pose[2] + bias
        print('grasp pose:', target_pose)
        send_pose(target_pose)
    else:
        raise ValueError('stop')
    grasp = input("grasp object now: [Y/N]")
    if grasp == 'Y' or grasp == 'y':
        close_finger(width=0.0)  
        # close_finger2() 
    else:  
        raise ValueError('stop')

    put = input("put the object to destination: [Y/N]")
    if put == 'Y' or put == 'y':
        target_pose[2] = 0.4
        send_pose(target_pose)
        send_joint_angles(end_position_joint_angles)
        # time.sleep(1)
        # close_finger(width=0.15)
        open_finger()
    else:
        raise ValueError('stop')
    
    # save data
    save_path = os.path.join('./dump/realtrain/scene_{}'.format(str(scene_id)))
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    num_file = len(os.listdir(save_path))
    file_dir = os.path.join(save_path, 'frame_{}'.format(num_file+1))
    if not os.path.exists(file_dir):
        os.makedirs(file_dir)                        
    np.save(os.path.join(save_path, 'frame_{}'.format(num_file+1), 'depth.npy'), depth)
    np.save(os.path.join(save_path, 'frame_{}'.format(num_file+1), 'refine_depth.npy'), refine_depth)
    np.save(os.path.join(save_path, 'frame_{}'.format(num_file+1), 'color.npy'), color)
    np.save(os.path.join(save_path, 'frame_{}'.format(num_file+1), 'grasp.npy'), gg)
    np.save(os.path.join(save_path, 'frame_{}'.format(num_file+1), 'intrinsic.npy'), camera)
    np.save(os.path.join(save_path, 'frame_{}'.format(num_file+1), 'pose.npy'), eepose)
