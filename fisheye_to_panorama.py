import os
import re
import cv2
import numpy as np
from tqdm import tqdm
from collections import defaultdict
from scipy.spatial.transform import Rotation as R

# Default polynomial fisheye model in RflySim (full ver. 3.07) (as per MATLAB calibration)
MappingCoefficients = [167.7745, -0.0020, 5.9066e-07, -7.6388e-09]
ImageSize = (640, 640)
DistortionCenter = (320, 320)

# Relative mounting angles of 4 cameras on UAV (front, right, back, left)
def get_camera_mount_rotations():
    def rotation_matrix_y(angle_deg):
        angle_rad = np.deg2rad(angle_deg)
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return [rotation_matrix_y(0), rotation_matrix_y(90), rotation_matrix_y(180), rotation_matrix_y(-90)]

# Polynomial and its derivative
def polynomial_r(theta, coeffs):
    return sum([k * theta ** (2 * i + 1) for i, k in enumerate(coeffs)])

def polynomial_r_derivative(theta, coeffs):
    return sum([(2 * i + 1) * k * theta ** (2 * i) for i, k in enumerate(coeffs)])

def inverse_polynomial_r(r, coeffs, theta_init=0.1, iter_num=10):
    theta = theta_init
    for _ in range(iter_num):
        f = polynomial_r(theta, coeffs) - r
        df = polynomial_r_derivative(theta, coeffs)
        theta -= f / (df + 1e-8)
    return theta

def apply_feathering(pano_img, weight_map, feather_size=60):
    feather_size = max(3, feather_size | 1)
    norm_weight = weight_map / (weight_map.max() + 1e-8)
    feathered_mask = cv2.GaussianBlur(norm_weight, (feather_size, feather_size), 0)
    feathered_mask[feathered_mask == 0] = 1e-8
    pano_img_feathered = pano_img.astype(np.float32) / feathered_mask[..., None]
    pano_img_feathered = np.clip(pano_img_feathered, 0, 255).astype(np.uint8)
    return pano_img_feathered

def stitch_fisheye_to_panorama(fisheye_imgs, coeffs, center, rotations, pano_size=(1024, 512)):
    pano_w, pano_h = pano_size
    pano_img = np.zeros((pano_h, pano_w, 3), dtype=np.float32)
    weight_map = np.zeros((pano_h, pano_w), dtype=np.float32)

    xs = np.linspace(0, pano_w - 1, pano_w)
    ys = np.linspace(0, pano_h - 1, pano_h)
    xv, yv = np.meshgrid(xs, ys)
    lon = (xv / pano_w) * 2 * np.pi - np.pi
    lat = np.pi / 2 - (yv / pano_h) * np.pi

    dir_x = np.sin(lon) * np.cos(lat)
    dir_y = np.sin(lat)
    dir_z = np.cos(lon) * np.cos(lat)
    directions = np.stack([dir_x, dir_y, dir_z], axis=2)

    for i, img in enumerate(fisheye_imgs):
        R_cam = rotations[i]
        invR = R_cam.T
        cam_dirs = directions @ invR.T

        x_cam, y_cam, z_cam = cam_dirs[:, :, 0], cam_dirs[:, :, 1], cam_dirs[:, :, 2]
        theta = np.arccos(np.clip(z_cam, -1.0, 1.0))
        r = polynomial_r(theta, coeffs)
        norm = np.sqrt(x_cam**2 + y_cam**2 + 1e-8)

        u = r * x_cam / norm + center[0]
        v = r * y_cam / norm + center[1]

        mask = (u >= 0) & (u < img.shape[1] - 1) & (v >= 0) & (v < img.shape[0] - 1) & (z_cam > 0)
        u_flat, v_flat = u[mask].astype(np.float32), v[mask].astype(np.float32)
        u0, v0 = np.floor(u_flat).astype(np.int32), np.floor(v_flat).astype(np.int32)
        u1, v1 = np.clip(u0 + 1, 0, img.shape[1] - 1), np.clip(v0 + 1, 0, img.shape[0] - 1)

        du = (u_flat - u0)[:, None]
        dv = (v_flat - v0)[:, None]

        c00 = img[v0, u0]
        c10 = img[v0, u1]
        c01 = img[v1, u0]
        c11 = img[v1, u1]

        interp = (
            (1 - du) * (1 - dv) * c00 +
            du * (1 - dv) * c10 +
            (1 - du) * dv * c01 +
            du * dv * c11
        )

        pano_img[mask] += interp
        weight_map[mask] += 1

    weight_map[weight_map == 0] = 1
    pano_img = (pano_img / weight_map[..., None]).clip(0, 255).astype(np.uint8)
    pano_img = np.rot90(pano_img, k=2)
    pano_img = apply_feathering(pano_img, weight_map)
    return pano_img

def group_images_by_timestamp(image_dir):
    pattern = re.compile(r"img_(\d)_(\d+\.\d+)\.jpg")
    groups = defaultdict(dict)
    for filename in os.listdir(image_dir):
        match = pattern.match(filename)
        if match:
            direction, timestamp = match.groups()
            groups[timestamp][int(direction)] = os.path.join(image_dir, filename)
    return [group for group in groups.values() if len(group) == 4]

def load_uav_rotation(label_dir, timestamp):
    label_path = os.path.join(label_dir, f"label_{timestamp}.txt")
    with open(label_path, 'r') as f:
        tokens = list(map(float, f.read().strip().split()))
    roll, pitch, yaw = np.deg2rad(tokens[4:7])  # Assuming degree input
    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

def process_groups(image_dir, output_dir, pano_size=(1280, 640)):
    os.makedirs(output_dir, exist_ok=True)
    groups = group_images_by_timestamp(image_dir)
    label_dir = image_dir  # assumes labels are in the same directory
    for group in tqdm(groups, desc="Processing panoramas"):
        try:
            imgs = [cv2.imread(group[i]) for i in range(4)]
            if any(img is None for img in imgs):
                print("Warning: Failed to load some images.", group)
                continue
            imgs = [cv2.resize(img, ImageSize) for img in imgs]
            timestamp = re.search(r"_(\d+\.\d+)\.jpg", os.path.basename(group[0])).group(1)
            R_uav = load_uav_rotation(label_dir, timestamp)
            cam_mounts = get_camera_mount_rotations()
            rotations = [R_uav @ mount for mount in cam_mounts]
            pano = stitch_fisheye_to_panorama(imgs, MappingCoefficients, DistortionCenter, rotations, pano_size)
            out_path = os.path.join(output_dir, f"panorama_{timestamp}.jpg")
            cv2.imwrite(out_path, pano)
        except Exception as e:
            print(f"Error processing group: {e}")

if __name__ == "__main__":
    image_dir = 'fisheye_view/scene010/traj001'
    output_dir = 'panorama_view/scene010/traj001'
    process_groups(image_dir, output_dir)



