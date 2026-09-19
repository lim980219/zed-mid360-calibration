import json
from pathlib import Path
import cv2
import numpy as np
from .bag_io import Bag, stamp, matrix_attr, decode_image, decode_points, write_pcd


def camera_parameters(msg, image):
    h, w = image.shape[:2]
    if (msg.width, msg.height) != (w, h):
        raise ValueError("CameraInfo resolution does not match image; do not reuse K from another resolution.")
    if msg.binning_x > 1 or msg.binning_y > 1 or msg.roi.x_offset or msg.roi.y_offset:
        raise ValueError("Binned/cropped CameraInfo is not supported; record full-resolution left images.")
    projection = matrix_attr(msg, "P").reshape(3, 4)
    rotation = matrix_attr(msg, "R").reshape(3, 3)
    if not np.isfinite(projection).all() or projection[0, 0] <= 0 or projection[1, 1] <= 0:
        raise ValueError("Invalid rectified projection matrix P in CameraInfo.")
    if not np.allclose(projection[:, 3], 0, atol=1e-7):
        raise ValueError("Nonzero stereo projection offset: choose the LEFT CameraInfo.")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) < 0.99:
        raise ValueError("CameraInfo.R must be a valid rectification rotation.")
    if not np.allclose(projection[2, :3], [0, 0, 1]) or abs(projection[0, 1]) > 1e-7:
        raise ValueError("Only a standard pinhole rectified P is supported.")
    return projection[:, :3], rotation


def motion_pixels(reference, other, min_features):
    # Sparse flow at <=640px width; report displacement at original resolution.
    scale = min(1.0, 640.0 / reference.shape[1])
    def gray(img):
        return cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None, fx=scale, fy=scale)
    a, b = gray(reference), gray(other)
    features = cv2.goodFeaturesToTrack(a, maxCorners=500, qualityLevel=0.01, minDistance=8)
    if features is None or len(features) < min_features:
        raise ValueError("Too few image features to check stationarity.")
    tracked, status, _ = cv2.calcOpticalFlowPyrLK(a, b, features, None)
    if tracked is None:
        raise ValueError("Optical flow failed.")
    back, back_status, _ = cv2.calcOpticalFlowPyrLK(b, a, tracked, None)
    if back is None:
        raise ValueError("Backward optical flow failed.")
    good = (status.ravel() != 0) & (back_status.ravel() != 0)
    good &= np.linalg.norm((back - features).reshape(-1, 2), axis=1) < 1.0
    if np.count_nonzero(good) < min_features:
        raise ValueError("Too few reliable tracks to check stationarity.")
    displacement = np.linalg.norm((tracked - features).reshape(-1, 2)[good], axis=1) / scale
    return float(np.percentile(displacement, 90))


def downsample(points, voxel):
    if not len(points):
        return points
    _, ids = np.unique(np.floor(points[:, :3] / voxel).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(ids)]


def prepare_scene(path, output, index, cfg):
    data = cfg["data"]
    topics = cfg["topics"]
    with Bag(path) as bag:
        for role, topic in topics.items():
            if topic not in bag.topics:
                raise ValueError("Missing " + role + " topic " + topic + "; available: " + ", ".join(bag.topics))
        if bag.topics[topics["image"]] not in ("sensor_msgs/Image", "sensor_msgs/CompressedImage"):
            raise ValueError("Image topic has an unsupported type.")
        if bag.topics[topics["camera_info"]] != "sensor_msgs/CameraInfo":
            raise ValueError("camera_info topic must contain CameraInfo.")
        point_type = bag.topics[topics["points"]]
        if point_type != "sensor_msgs/PointCloud2" and not point_type.endswith("/CustomMsg"):
            raise ValueError("Unsupported lidar message type: " + point_type)
        midpoint = (bag.start + bag.end) / 2
        half = min(float(data["accumulation_seconds"]) / 2, (bag.end - bag.start) / 2)
        if half < 0.1:
            raise ValueError("Bag is too short.")
        lo, hi = midpoint - half, midpoint + half
        # A bounded reservoir at 11 evenly spaced recording times.
        targets = np.linspace(lo, hi, 11)
        samples = [None] * len(targets)
        distances = np.full(len(targets), np.inf)
        camera_info = None
        camera_signature = None
        for topic, msg, rec in bag.messages([topics["image"], topics["camera_info"]]):
            if topic == topics["camera_info"]:
                signature = (msg.width, msg.height, msg.header.frame_id,
                             tuple(matrix_attr(msg, "P").ravel()), tuple(matrix_attr(msg, "R").ravel()))
                if camera_signature is not None and signature != camera_signature:
                    raise ValueError("CameraInfo changes within a bag.")
                camera_signature, camera_info = signature, msg
            elif lo <= rec <= hi:
                delta = abs(targets - rec)
                for i in np.flatnonzero(delta < distances):
                    distances[i], samples[i] = delta[i], (msg, rec)
        if camera_info is None or any(item is None for item in samples):
            raise ValueError("No image/CameraInfo in the selected time window.")
        if max(distances) > max(0.25, half * 0.2):
            raise ValueError("Images do not cover the accumulation window.")
        image_msg, image_record = samples[len(samples) // 2]
        image = decode_image(image_msg)
        intrinsic, rectification = camera_parameters(camera_info, image)
        camera_frame = image_msg.header.frame_id
        if not camera_frame or camera_frame != camera_info.header.frame_id:
            raise ValueError("Image and CameraInfo must use the same nonempty left optical frame.")
        image_time = stamp(image_msg) + data["camera_time_offset_seconds"]
        motion = 0.0
        for sample, _ in samples:
            if sample.header.frame_id != camera_frame:
                raise ValueError("Camera frame changed.")
            other = decode_image(sample)
            if other.shape != image.shape:
                raise ValueError("Image resolution changed.")
            motion = max(motion, motion_pixels(image, other, data["min_tracked_features"]))
        if motion > data["max_motion_pixels"]:
            raise ValueError("Scene is moving: image flow {:.2f}px > {:.2f}px. Use stationary recordings.".format(
                motion, data["max_motion_pixels"]))
        points = np.empty((0, 4), np.float32)
        nearest = float("inf")
        lidar_frame = None
        cloud_count = 0
        first_time, last_time = float("inf"), -float("inf")
        for _, msg, rec in bag.messages([topics["points"]], lo, hi):
            t = stamp(msg)
            nearest = min(nearest, abs(t - image_time))
            if abs(t - image_time) > half + data["max_sync_seconds"]:
                continue
            if not msg.header.frame_id or (lidar_frame is not None and lidar_frame != msg.header.frame_id):
                raise ValueError("Point cloud frame is empty or changes.")
            lidar_frame = msg.header.frame_id
            cloud = decode_points(msg)
            valid = np.isfinite(cloud).all(axis=1)
            ranges = np.linalg.norm(cloud[:, :3], axis=1)
            valid &= (ranges >= data["min_range_m"]) & (ranges <= data["max_range_m"])
            cloud = downsample(cloud[valid], data["voxel_size_m"])
            points = downsample(np.concatenate([points, cloud]), data["voxel_size_m"])
            if len(points) > data["max_points"]:
                raise ValueError("Accumulated cloud exceeds max_points; increase voxel_size_m or shorten accumulation_seconds.")
            cloud_count += 1
            first_time, last_time = min(first_time, t), max(last_time, t)
        if nearest > data["max_sync_seconds"]:
            raise ValueError("Image/lidar header time gap {:.3f}s exceeds max_sync_seconds.".format(nearest))
        if cloud_count < 2 or last_time - first_time < half:
            raise ValueError("Lidar does not cover enough of the selected accumulation window.")
        if len(points) < data["min_points"]:
            raise ValueError("Not enough valid lidar points after filtering.")
        if camera_frame == lidar_frame:
            raise ValueError("Camera and lidar frame IDs must differ.")
    name = str(index)
    if not cv2.imwrite(str(output / "image" / (name + ".bmp")), image):
        raise OSError("Cannot write image.")
    write_pcd(output / "pcd" / (name + ".pcd"), points)
    np.savez_compressed(output / "samples" / (name + ".npz"), points=points, K=intrinsic, R=rectification)
    return {
        "index": index, "bag": str(Path(path).resolve()), "points": len(points), "cloud_messages": cloud_count,
        "image_header_time": image_time, "image_record_time": image_record,
        "nearest_time_gap_seconds": nearest, "motion_p90_pixels": motion,
        "lidar_frame": lidar_frame, "camera_frame": camera_frame,
        "width": image.shape[1], "height": image.shape[0],
        "K": intrinsic.tolist(), "R_rectification": rectification.tolist(),
    }


def prepare(paths, output, cfg):
    for name in ("image", "pcd", "samples"):
        (output / name).mkdir(parents=True)
    scenes = []
    for index, path in enumerate(paths):
        print("[prepare {}/{}] {}".format(index + 1, len(paths), path), flush=True)
        scene = prepare_scene(path, output, index, cfg)
        if scenes:
            for key in ("K", "R_rectification", "width", "height", "camera_frame", "lidar_frame"):
                if scene[key] != scenes[0][key]:
                    raise ValueError("All bags must use the same sensor frames/resolution/intrinsics: " + key)
        scenes.append(scene)
    (output / "manifest.json").write_text(json.dumps(scenes, indent=2), encoding="utf-8")
    return scenes

