import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import pytest
import yaml
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from zed_mid360_calibration.config import DEFAULT, load_config
from zed_mid360_calibration.geometry import mount_transform, project
from zed_mid360_calibration.bag_io import decode_points, decode_image, write_pcd
from zed_mid360_calibration.backend import initial_transform, parse_matrix, check_log
from zed_mid360_calibration.cli import main

STORE = get_typestore(Stores.ROS1_NOETIC)
TYPES = STORE.types


def synthetic_bag(path, moving=False, offset=0.0, custom=False):
    rng = np.random.default_rng(42)
    base = rng.integers(0, 256, (120, 160, 3), dtype=np.uint8)
    points = np.column_stack([rng.uniform(2, 6, 400), rng.uniform(-1, 1, 400),
                              rng.uniform(-1, 1, 400), rng.uniform(0, 255, 400)]).astype("<f4")
    k = np.array([[100., 0, 80], [0, 100, 60], [0, 0, 1]])
    if custom:
        STORE.register(get_types_from_msg(
            "uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line",
            "livox_ros_driver2/msg/CustomPoint"))
        STORE.register(get_types_from_msg(
            "std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nlivox_ros_driver2/CustomPoint[] points",
            "livox_ros_driver2/msg/CustomMsg"))
    point_type = "livox_ros_driver2/msg/CustomMsg" if custom else "sensor_msgs/msg/PointCloud2"
    with Writer(path) as writer:
        conns = {}
        for key, kind in [("image", "sensor_msgs/msg/Image"), ("camera_info", "sensor_msgs/msg/CameraInfo"),
                          ("points", point_type)]:
            conns[key] = writer.add_connection(DEFAULT["topics"][key], kind, typestore=STORE)
        for i in range(21):
            t = 100 + i * 0.1
            def header(frame, extra=0):
                ns = round((t + extra) * 1e9)
                stamp = TYPES["builtin_interfaces/msg/Time"](ns // 10**9, ns % 10**9)
                return TYPES["std_msgs/msg/Header"](i, stamp, frame)
            image = np.roll(base, i * 2, axis=1) if moving else base
            image_msg = TYPES["sensor_msgs/msg/Image"](header("left_optical", offset), 120, 160,
                         "bgr8", 0, 480, image.ravel())
            roi = TYPES["sensor_msgs/msg/RegionOfInterest"](0, 0, 0, 0, False)
            info = TYPES["sensor_msgs/msg/CameraInfo"](header("left_optical"), 120, 160, "plumb_bob",
                   np.zeros(5), k.ravel(), np.eye(3).ravel(), np.column_stack([k, np.zeros(3)]).ravel(), 0, 0, roi)
            if custom:
                ps = [STORE.types["livox_ros_driver2/msg/CustomPoint"](0, float(p[0]), float(p[1]), float(p[2]),
                      int(p[3]), 0, 0) for p in points]
                cloud = STORE.types[point_type](header("livox"), round(t * 1e9), len(ps), 0,
                                                np.zeros(3, np.uint8), ps)
            else:
                fields = [TYPES["sensor_msgs/msg/PointField"](n, j * 4, 7, 1)
                          for j, n in enumerate(["x", "y", "z", "intensity"])]
                cloud = TYPES[point_type](header("livox"), 1, len(points), fields, False, 16,
                         len(points) * 16, points.view(np.uint8).ravel(), True)
            for key, msg in [("image", image_msg), ("camera_info", info), ("points", cloud)]:
                writer.write(conns[key], round(t * 1e9), STORE.serialize_ros1(msg, msg.__msgtype__))
    return points


@pytest.fixture
def config(tmp_path):
    cfg = copy.deepcopy(DEFAULT)
    # Synthetic values only; these are not the user's mounting measurements.
    cfg["camera_mount"].update(forward_m=0.12, left_m=0.04, down_m=0.08)
    cfg["data"].update(accumulation_seconds=1.5, min_points=100, min_tracked_features=10)
    cfg["quality"]["min_projected_points"] = 50
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    return path, cfg


def test_mount_axes_and_inverse():
    t = mount_transform(0.12, 0.04, 0.08, 10)
    np.testing.assert_allclose(t[:3, 3], [0.12, 0.04, -0.08])
    np.testing.assert_allclose(t[:3, 2], [np.cos(np.deg2rad(10)), 0, -np.sin(np.deg2rad(10))], atol=1e-15)
    np.testing.assert_allclose(t[:3, 0], [0, -1, 0], atol=1e-15)
    inv = np.linalg.inv(t)
    np.testing.assert_allclose(inv[:3, 3], -t[:3, :3].T @ t[:3, 3])
    np.testing.assert_allclose(inv @ t, np.eye(4), atol=1e-15)


def test_config_requires_real_mount(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(DEFAULT))
    with pytest.raises(ValueError, match="camera_mount.forward_m"):
        load_config(path)


def test_padded_big_endian_cloud_and_image():
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=[">f4"] * 3, offsets=[0, 4, 8], itemsize=16))
    buffer = bytearray(80)
    array = np.ndarray((2, 2), dtype=dtype, buffer=buffer, strides=(40, 16))
    for j, key in enumerate(["x", "y", "z"]):
        array[key] = np.arange(4).reshape(2, 2) + j
    msg = NS(fields=[NS(name=n, offset=i * 4, count=1, datatype=7) for i, n in enumerate(["x", "y", "z"])],
             point_step=16, row_step=40, width=2, height=2, is_bigendian=True, data=buffer)
    decoded = decode_points(msg)
    np.testing.assert_array_equal(decoded[:, 0], [0, 1, 2, 3])
    np.testing.assert_array_equal(decoded[:, 3], 0)
    image = decode_image(NS(encoding="rgb8", step=8, width=2, height=1,
                           data=bytes([255, 0, 0, 0, 255, 0, 99, 99])))
    np.testing.assert_array_equal(image[0], [[0, 0, 255], [0, 255, 0]])


@pytest.mark.parametrize("custom", [False, True])
def test_prepare_real_serialized_bag(tmp_path, config, custom):
    config_path, cfg = config
    bag = tmp_path / "scene_8.bag"
    synthetic_bag(bag, custom=custom)
    output = tmp_path / "out"
    assert main([str(bag), "--config", str(config_path), "--output", str(output), "--prepare-only"]) == 0
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest[0]["index"] == 0  # Original filenames need not be consecutive.
    assert manifest[0]["points"] >= 100
    assert (output / "pcd/0.pcd").read_bytes().split(b"DATA binary\n")[0].endswith(b"POINTS 400\n")
    params = yaml.safe_load((output / "multi_calib.yaml").read_text())
    assert params["camera"]["dist_coeffs"] == [0.] * 5
    cvfile = cv2.FileStorage(str(output / "backend_config.yaml"), cv2.FILE_STORAGE_READ)
    np.testing.assert_allclose(cvfile.getNode("ExtrinsicMat").mat(), initial_transform(cfg))
    cvfile.release()
    # Preserve existing output rather than silently reuse/overwrite stale estimates.
    assert main([str(bag), "--config", str(config_path), "--output", str(output), "--prepare-only"]) == 1


@pytest.mark.parametrize("moving,offset,expected", [(True, 0, "moving"), (False, 10, "time gap")])
def test_bad_bag_rejected(tmp_path, config, moving, offset, expected):
    config_path, _ = config
    bag = tmp_path / "bad.bag"
    synthetic_bag(bag, moving=moving, offset=offset)
    output = tmp_path / "out"
    assert main([str(bag), "--config", str(config_path), "--output", str(output), "--prepare-only"]) == 1
    assert expected in json.loads((output / "failure.json").read_text())["error"]
    assert not (output / "extrinsics.yaml").exists()


def test_backend_checks(tmp_path):
    path = tmp_path / "extrinsic.txt"
    np.savetxt(path, np.eye(4), delimiter=",")
    np.testing.assert_allclose(parse_matrix(path), np.eye(4))
    path.write_text("nan")
    with pytest.raises(ValueError):
        parse_matrix(path)
    for log in ["", "pnp size: 0", "pnp size: 40\nTermination: FAILURE"]:
        with pytest.raises(ValueError):
            check_log(log, 30)
    assert check_log("pnp size: 40\npnp size: 50", 30)["min_correspondences"] == 40


def test_pipeline_with_fake_optimizer(tmp_path, config, monkeypatch):
    from zed_mid360_calibration import cli
    config_path, cfg = config
    bag = tmp_path / "test.bag"
    synthetic_bag(bag)
    expected = initial_transform(cfg)
    monkeypatch.setattr(cli, "run_backend", lambda output, c: (expected, {"min_correspondences": 100}))
    output = tmp_path / "out"
    assert main([str(bag), "--config", str(config_path), "--output", str(output)]) == 0
    report = yaml.safe_load((output / "extrinsics.yaml").read_text())
    assert report["status"] == "estimated_unvalidated"
    np.testing.assert_allclose(report["T_camera_lidar"], expected)
    tf_node = ET.parse(output / "static_tf.launch").getroot().find("node")
    args = tf_node.attrib["args"].split()
    np.testing.assert_allclose(np.array(args[:3], float), [0.12, 0.04, -0.08])
    assert args[-2:] == ["livox", "left_optical"]
    assert (output / "overlays/0_result.png").exists()


def test_projection_discards_behind_camera():
    points = np.array([[0, 0, 2], [0, 0, -2], [200, 0, 2]])
    pixels, _, ids = project(points, np.eye(4), np.array([[100, 0, 80], [0, 100, 60], [0, 0, 1]]), 160, 120)
    np.testing.assert_array_equal(ids, [0])
    np.testing.assert_allclose(pixels, [[80, 60]])



def test_rectified_output_converts_back_to_physical_camera(tmp_path, config):
    from scipy.spatial.transform import Rotation
    from zed_mid360_calibration.report import save_report
    _, cfg = config
    (tmp_path / "samples").mkdir()
    (tmp_path / "image").mkdir()
    optical = initial_transform(cfg)
    rect = np.eye(4)
    rect[:3, :3] = Rotation.from_euler("z", 2, degrees=True).as_matrix()
    k = np.array([[100., 0, 80], [0, 100, 60], [0, 0, 1]])
    points = np.tile([3., 0., 0., 100.], (200, 1))
    np.savez(tmp_path / "samples/0.npz", points=points, K=k)
    cv2.imwrite(str(tmp_path / "image/0.bmp"), np.zeros((120, 160, 3), np.uint8))
    scene = {"index": 0, "R_rectification": rect[:3, :3].tolist(),
             "lidar_frame": "livox", "camera_frame": "left_optical"}
    report = save_report(tmp_path, [scene], cfg, optical, rect @ optical, {})
    np.testing.assert_allclose(report["T_camera_lidar"], optical, atol=1e-14)
    np.testing.assert_allclose(report["T_lidar_camera"], np.linalg.inv(optical), atol=1e-14)


def test_backend_process_completion_and_failure(tmp_path, config, monkeypatch):
    # Exercise supervision without ROS. Process diagnostics/result files follow HKU's contract.
    from zed_mid360_calibration import backend
    _, cfg = config
    cfg["backend"]["headless"] = False
    monkeypatch.setattr(backend, "os", NS(name="posix", environ={"DISPLAY": ":99"}))
    monkeypatch.setattr(backend.shutil, "which", lambda name: "/usr/bin/" + name)
    class FakeProcess:
        pid = 1234
        returncode = None
        def poll(self):
            return self.returncode
    stopped = []
    def popen(command, **kwargs):
        np.savetxt(tmp_path / "backend_extrinsic.txt", np.eye(4), delimiter=",")
        kwargs["stdout"].write("pnp size: 50\npush enter to publish again\n")
        kwargs["stdout"].flush()
        assert kwargs["env"]["ROS_MASTER_URI"].startswith("http://127.0.0.1:")
        assert kwargs["start_new_session"] is True
        return FakeProcess()
    monkeypatch.setattr(backend.subprocess, "Popen", popen)
    monkeypatch.setattr(backend, "stop_process", lambda p: stopped.append(p))
    result, checks = backend.run_backend(tmp_path, cfg)
    assert len(stopped) == 1
    assert checks["min_correspondences"] == 50
    np.testing.assert_allclose(result, np.eye(4))


def test_backend_timeout_stops_owned_process(tmp_path, config, monkeypatch):
    from zed_mid360_calibration import backend
    _, cfg = config
    cfg["backend"].update(headless=False, timeout_seconds=0.001)
    monkeypatch.setattr(backend, "os", NS(name="posix", environ={"DISPLAY": ":99"}))
    monkeypatch.setattr(backend.shutil, "which", lambda name: "/usr/bin/" + name)
    fake = NS(poll=lambda: None)
    stopped = []
    monkeypatch.setattr(backend.subprocess, "Popen", lambda *a, **kw: fake)
    monkeypatch.setattr(backend, "stop_process", lambda p: stopped.append(p))
    with pytest.raises(TimeoutError):
        backend.run_backend(tmp_path, cfg)
    assert stopped == [fake]

