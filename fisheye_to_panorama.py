import os
import re
import cv2
import numpy as np
from tqdm import tqdm
from collections import defaultdict

# Defalut Polynomial fisheye Model in RflySim (ver. ) (as per MATLAB calibration)
MappingCoefficients = [167.7745, -0.0020, 5.9066e-07, -7.6388e-09]
ImageSize = (640, 640)
DistortionCenter = (321.0257, 321.2249)

# Rotation matrix around Y axis
def rotation_matrix_y(angle_deg):
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

# Direction of each camera (front, right, rear, left)
rotations = [
    rotation_matrix_y(0),
    rotation_matrix_y(90),
    rotation_matrix_y(180),
    rotation_matrix_y(-90),
]

# Polynomial and its derivative
def polynomial_r(theta, coeffs):
    return sum([k * theta ** (2 * i + 1) for i, k in enumerate(coeffs)])

def polynomial_r_derivative(theta, coeffs):
    return sum([(2 * i + 1) * k * theta ** (2 * i) for i, k in enumerate(coeffs)])

# Invert the polynomial mapping using Newton-Raphson
def inverse_polynomial_r(r, coeffs, theta_init=0.1, iter_num=10):
    theta = theta_init
    for _ in range(iter_num):
        f = polynomial_r(theta, coeffs) - r
        df = polynomial_r_derivative(theta, coeffs)
        theta -= f / (df + 1e-8)
    return theta

# Compute 3D direction from 2D image coordinates
def pixel_to_direction(u, v, center, coeffs):
    cx, cy = center
    dx, dy = u - cx, v - cy
    r = np.sqrt(dx**2 + dy**2)
    if r < 1e-8:
        return np.array([0, 0, 1])
    theta = inverse_polynomial_r(r, coeffs)
    sin_theta = np.sin(theta)
    x = sin_theta * dx / r
    y = sin_theta * dy / r
    z = np.cos(theta)
    return np.array([x, y, z]) / np.linalg.norm([x, y, z])

# Convert direction to spherical coordinates
def direction_to_equirectangular_coords(vec):
    x, y, z = vec
    lon = np.arctan2(x, z)
    lat = np.arcsin(y)
    return lon, lat

# Convert spherical coordinates to panorama pixel positions
def equirectangular_coords_to_pixel(lon, lat, width, height):
    x = (lon + np.pi) / (2 * np.pi) * width
    y = (np.pi / 2 - lat) / np.pi * height
    return x, y

# Main function: stitch 4 fisheye images into a panorama
def stitch_fisheye_to_panorama(fisheye_imgs, coeffs, center, rotations, pano_size=(1024, 512)):
    pano_w, pano_h = pano_size
    pano_img = np.zeros((pano_h, pano_w, 3), dtype=np.float32)
    weight_map = np.zeros((pano_h, pano_w), dtype=np.float32)

    # Create equirectangular grid of directions
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
        R = rotations[i]
        invR = R.T
        cam_dirs = directions @ invR.T

        x_cam, y_cam, z_cam = cam_dirs[:,:,0], cam_dirs[:,:,1], cam_dirs[:,:,2]
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
    return pano_img

# Group images by timestamp based on naming convention img_<id>_<timestamp>.jpg
def group_images_by_timestamp(image_dir):
    pattern = re.compile(r"img_(\d)_(\d+\.\d+)\.jpg")
    groups = defaultdict(dict)
    for filename in os.listdir(image_dir):
        match = pattern.match(filename)
        if match:
            direction, timestamp = match.groups()
            groups[timestamp][int(direction)] = os.path.join(image_dir, filename)
    return [group for group in groups.values() if len(group) == 4]

# Process all image groups and save panorama outputs
def process_groups(image_dir, output_dir, pano_size=(2048, 1024)):
    os.makedirs(output_dir, exist_ok=True)
    groups = group_images_by_timestamp(image_dir)
    for group in tqdm(groups, desc="Processing panoramas"):
        try:
            imgs = [cv2.imread(group[i]) for i in range(4)]
            if any(img is None for img in imgs):
                print("Warning: Failed to load some images.", group)
                continue
            imgs = [cv2.resize(img, ImageSize) for img in imgs]
            pano = stitch_fisheye_to_panorama(imgs, MappingCoefficients, DistortionCenter, rotations, pano_size)
            timestamp = re.search(r"_(\d+\.\d+)\.jpg", os.path.basename(group[0])).group(1)
            out_path = os.path.join(output_dir, f"panorama_{timestamp}.jpg")
            cv2.imwrite(out_path, pano)
        except Exception as e:
            print(f"Error processing group: {e}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Stitch 4 fisheye images into a panorama")
    parser.add_argument("--input_dir", type=str, required=True, help="Directory containing fisheye images")
    parser.add_argument("--output_dir", type=str, required=True, help="Directory to save panoramas")
    args = parser.parse_args()
    process_groups(args.input_dir, args.output_dir)