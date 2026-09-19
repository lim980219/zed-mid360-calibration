import json
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from .geometry import pose_vector, project


def overlay(image, points, transform, intrinsic):
    canvas = image.copy()
    pixels, depth, _ = project(points, transform, intrinsic, image.shape[1], image.shape[0])
    if len(pixels):
        # Far-to-near painting gives nearer points precedence at shared pixels.
        ids = np.argsort(-depth)
        ids = ids[::max(1, len(ids) // 50000)]
        colors = cv2.applyColorMap(np.uint8(np.clip(depth[ids] / 30.0, 0, 1) * 255), cv2.COLORMAP_TURBO).reshape(-1, 3)
        for pixel, color in zip(pixels[ids], colors):
            cv2.circle(canvas, tuple(pixel.astype(int)), 1, tuple(int(x) for x in color), -1)
    return canvas, len(pixels)


def save_report(output, scenes, cfg, initial, rectified_result, diagnostics):
    rect = np.eye(4)
    rect[:3, :3] = np.array(scenes[0]["R_rectification"])
    result = np.linalg.inv(rect) @ rectified_result
    inverse = np.linalg.inv(result)
    # Compare camera centres in lidar coordinates, and their orientation.
    initial_inverse = np.linalg.inv(initial)
    translation_change = float(np.linalg.norm(inverse[:3, 3] - initial_inverse[:3, 3]))
    rotation_change = float(np.degrees(Rotation.from_matrix(
        inverse[:3, :3] @ initial_inverse[:3, :3].T).magnitude()))
    issues = []
    if translation_change > cfg["quality"]["max_translation_change_m"]:
        issues.append("Translation changed excessively from the configured mount.")
    if rotation_change > cfg["quality"]["max_rotation_change_deg"]:
        issues.append("Rotation changed excessively from the configured mount.")
    (output / "overlays").mkdir()
    projections = []
    for scene in scenes:
        name = str(scene["index"])
        image = cv2.imread(str(output / "image" / (name + ".bmp")))
        with np.load(str(output / "samples" / (name + ".npz"))) as sample:
            before, _ = overlay(image, sample["points"], rect @ initial, sample["K"])
            after, count = overlay(image, sample["points"], rectified_result, sample["K"])
        for suffix, img in (("initial", before), ("result", after)):
            if not cv2.imwrite(str(output / "overlays" / (name + "_" + suffix + ".png")), img):
                raise OSError("Cannot write projection image.")
        projections.append({"scene": scene["index"], "projected_points": count})
        if count < cfg["quality"]["min_projected_points"]:
            issues.append("Too little projected overlap in scene " + name)
    status = "needs_review" if issues else "estimated_unvalidated"
    report = {
        "status": status, "units": "metres", "quaternion_order": "x y z w",
        "equation": "p_camera_optical = T_camera_lidar @ p_lidar",
        "lidar_frame": scenes[0]["lidar_frame"], "camera_optical_frame": scenes[0]["camera_frame"],
        "T_camera_lidar": result.tolist(), "T_lidar_camera": inverse.tolist(),
        "T_rectified_camera_lidar": rectified_result.tolist(),
        "camera_pose_in_lidar_xyz_xyzw": pose_vector(inverse),
        "lidar_pose_in_camera_xyz_xyzw": pose_vector(result),
        "translation_change_m": translation_change, "rotation_change_deg": rotation_change,
        "backend": diagnostics, "projection": projections, "issues": issues,
        "validation_note": "Sanity checks and training-scene overlays are not an accuracy measurement. "
                           "Validate independently before publishing TF.",
    }
    for name, matrix in (("T_camera_lidar.txt", result), ("T_lidar_camera.txt", inverse)):
        np.savetxt(str(output / name), matrix, fmt="%.12g")
    (output / "extrinsics.yaml").write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    # ROS tf2 publisher arguments specify the CHILD pose in PARENT coordinates.
    root = ET.Element("launch")
    root.append(ET.Comment(" Review overlays and validate the estimate before publishing. "))
    args = " ".join("{:.12g}".format(x) for x in pose_vector(inverse))
    args += " " + scenes[0]["lidar_frame"] + " " + scenes[0]["camera_frame"]
    ET.SubElement(root, "node", pkg="tf2_ros", type="static_transform_publisher",
                  name="mid360_zed_left_extrinsic", args=args)
    ET.ElementTree(root).write(str(output / "static_tf.launch"), encoding="utf-8", xml_declaration=True)
    return report

