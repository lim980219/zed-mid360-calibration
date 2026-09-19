"""Generate HKU inputs and supervise only the ROS process tree we start."""
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .geometry import mount_transform


def initial_transform(cfg):
    m = cfg["camera_mount"]
    return np.linalg.inv(mount_transform(m["forward_m"], m["left_m"], m["down_m"],
                                        m["pitch_down_deg"], m["yaw_left_deg"], m["roll_deg"]))


def write_inputs(output, scenes, cfg):
    physical = initial_transform(cfg)  # p_optical = T_optical_lidar p_lidar
    rectification = np.eye(4)
    rectification[:3, :3] = np.array(scenes[0]["R_rectification"])
    initial = rectification @ physical
    calibration = output / "backend_config.yaml"
    text = "%YAML:1.0\n---\nExtrinsicMat: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: d\n  data: "
    text += json.dumps(initial.ravel().tolist()) + "\n"
    text += 'PointCloudTopic: "/livox/lidar"\nImageTopic: "/zed2/zed_node/left/image_rect_color"\n'
    text += "\n".join("{}: {}".format(k, v) for k, v in cfg["edge"].items()) + "\n"
    calibration.write_text(text, encoding="utf-8")
    parameters = {
        "common": {"image_path": str(output / "image"), "pcd_path": str(output / "pcd"),
                   "result_path": str(output / "backend_extrinsic.txt"), "data_num": len(scenes)},
        "camera": {"camera_matrix": np.array(scenes[0]["K"]).ravel().tolist(), "dist_coeffs": [0.] * 5},
        "calib": {"calib_config_file": str(calibration),
                  "use_rough_calib": cfg["backend"]["use_rough_calib"]},
    }
    params = output / "multi_calib.yaml"
    params.write_text(yaml.safe_dump(parameters), encoding="utf-8")
    root = ET.Element("launch")
    group = ET.SubElement(root, "group", ns="zed_mid360_offline")
    ET.SubElement(group, "rosparam", command="load", file=str(params))
    ET.SubElement(group, "node", pkg="livox_camera_calib", type="lidar_camera_multi_calib",
                  name="calibrator", output="screen", required="true")
    ET.ElementTree(root).write(str(output / "backend.launch"), encoding="utf-8", xml_declaration=True)
    np.savetxt(str(output / "initial_T_camera_lidar.txt"), physical, fmt="%.12g")
    return physical


def parse_matrix(path):
    matrix = np.loadtxt(str(path), delimiter=",")
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Backend result is not a finite 4x4 matrix.")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError("Invalid homogeneous bottom row.")
    rot = matrix[:3, :3]
    if not np.allclose(rot.T @ rot, np.eye(3), atol=1e-4) or abs(np.linalg.det(rot) - 1) > 1e-4:
        raise ValueError("Backend returned an invalid rotation.")
    # The upstream text file rounds rotations to six significant digits.
    u, _, vt = np.linalg.svd(rot)
    matrix[:3, :3] = u @ vt
    return matrix


def check_log(text, min_correspondences):
    counts = [int(n) for n in re.findall(r"pnp size:\s*(\d+)", text)]
    if not counts:
        raise ValueError("No correspondence diagnostics in backend log; cannot validate this backend version.")
    if min(counts) < min_correspondences:
        raise ValueError("Insufficient edge correspondences: minimum {} < {}.".format(min(counts), min_correspondences))
    if re.search(r"Termination:\s*FAILURE|terminate called|Segmentation fault|FATAL", text, re.I):
        raise ValueError("Backend reported failure; inspect backend.log.")
    return {"min_correspondences": min(counts), "optimization_passes": len(counts)}


def stop_process(process):
    # A new POSIX session contains roslaunch, its private master, and calibration node.
    if os.name == "posix":
        for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            # Parent exit alone does not prove children exited.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
    else:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    if process.stdin:
        process.stdin.close()


def run_backend(output, cfg):
    if os.name != "posix" or shutil.which("roslaunch") is None:
        raise RuntimeError("Calibration requires Ubuntu/ROS1 Noetic with livox_camera_calib sourced. "
                           "Use --prepare-only for offline extraction on this system.")
    env = os.environ.copy()
    env.pop("ROS_NAMESPACE", None)
    env.pop("ROS_HOSTNAME", None)
    env["ROS_IP"] = "127.0.0.1"
    # Allocate an isolated master so this run cannot overwrite a robot's ROS parameters.
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    env["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(port)
    env["ROS_LOG_DIR"] = str(output / "ros_logs")
    command = ["roslaunch", "--port", str(port), str(output / "backend.launch")]
    if cfg["backend"]["headless"]:
        if shutil.which("xvfb-run") is None:
            raise RuntimeError("Headless HKU calibration needs: sudo apt install xvfb xauth")
        command = ["xvfb-run", "-a"] + command
    elif not env.get("DISPLAY"):
        raise RuntimeError("No DISPLAY. Set backend.headless: true.")
    (output / "backend_command.json").write_text(json.dumps(command, indent=2), encoding="utf-8")
    deadline = time.monotonic() + cfg["backend"]["timeout_seconds"]
    log_path = output / "backend.log"
    result_path = output / "backend_extrinsic.txt"
    if result_path.exists():
        raise ValueError("Refusing stale backend output; use a new output directory.")
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, env=env, stdin=subprocess.PIPE, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        next_progress = time.monotonic() + 30
        try:
            while True:
                text = log_path.read_text(encoding="utf-8", errors="replace")
                completed = "push enter to publish again" in text
                exited = process.poll() is not None
                if completed or exited:
                    if exited and process.returncode != 0:
                        raise RuntimeError("Backend exited with code {}; see {}".format(process.returncode, log_path))
                    if not result_path.exists():
                        raise RuntimeError("Backend did not produce extrinsic parameters; see " + str(log_path))
                    matrix = parse_matrix(result_path)
                    diagnostics = check_log(text, cfg["backend"]["min_correspondences"])
                    return matrix, diagnostics
                if time.monotonic() >= deadline:
                    raise TimeoutError("Calibration timeout; see " + str(log_path))
                if time.monotonic() >= next_progress:
                    print("[calibrate] Still running; log: " + str(log_path), flush=True)
                    next_progress = time.monotonic() + 30
                time.sleep(0.25)
        finally:
            stop_process(process)

