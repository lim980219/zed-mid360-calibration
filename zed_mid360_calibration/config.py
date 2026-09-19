from pathlib import Path
import copy
import math
import yaml

DEFAULT = {
    "camera_mount": {"forward_m": None, "left_m": None, "down_m": None,
                     "pitch_down_deg": 10.0, "yaw_left_deg": 0.0, "roll_deg": 0.0},
    "topics": {"image": "/zed2/zed_node/left/image_rect_color",
               "camera_info": "/zed2/zed_node/left/camera_info", "points": "/livox/lidar"},
    "data": {"image_is_rectified": True, "accumulation_seconds": 5.0,
             "max_sync_seconds": 0.1, "camera_time_offset_seconds": 0.0,
             "voxel_size_m": 0.02, "min_range_m": 0.5, "max_range_m": 40.0,
             "min_points": 1000, "max_points": 1000000,
             "max_motion_pixels": 2.0, "min_tracked_features": 30},
    "backend": {"timeout_seconds": 1800, "use_rough_calib": True, "headless": True,
                "min_correspondences": 30},
    "quality": {"max_translation_change_m": 0.25, "max_rotation_change_deg": 15.0,
                "min_projected_points": 100},
    "edge": {"Canny.gray_threshold": 10, "Canny.len_threshold": 200,
             "Voxel.size": 0.5, "Voxel.down_sample_size": 0.02,
             "Plane.min_points_size": 30, "Plane.normal_theta_min": 45,
             "Plane.normal_theta_max": 135, "Plane.max_size": 8,
             "Ransac.dis_threshold": 0.02, "Edge.min_dis_threshold": 0.03,
             "Edge.max_dis_threshold": 0.06},
}


def load_config(path):
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(raw, dict):
        raise ValueError("Config must be a YAML mapping.")
    result = copy.deepcopy(DEFAULT)
    for section, values in raw.items():
        if section not in result or not isinstance(values, dict):
            raise ValueError("Unknown/invalid config section: " + str(section))
        for key, value in values.items():
            if key not in result[section]:
                raise ValueError("Unknown config key: " + section + "." + key)
            result[section][key] = value
    for section, values in result.items():
        for key, value in values.items():
            default = DEFAULT[section][key]
            if section == "topics":
                if not isinstance(value, str) or not value.startswith("/"):
                    raise ValueError("Set an absolute ROS topic: topics." + key)
            elif isinstance(default, bool):
                if not isinstance(value, bool):
                    raise ValueError(section + "." + key + " must be a YAML boolean.")
            else:
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("Set a finite number for " + section + "." + key)
                if isinstance(default, int) and not isinstance(default, bool) and int(value) != value:
                    raise ValueError(section + "." + key + " must be an integer.")
                if section not in ("camera_mount",) and key != "camera_time_offset_seconds" and value <= 0:
                    raise ValueError(section + "." + key + " must be positive.")
    if result["data"]["min_range_m"] >= result["data"]["max_range_m"]:
        raise ValueError("min_range_m must be smaller than max_range_m.")
    if result["data"]["min_points"] > result["data"]["max_points"]:
        raise ValueError("min_points exceeds max_points.")
    if not result["data"]["image_is_rectified"]:
        raise ValueError("This package requires ZED left rectified images. Set the rectified image topic.")
    return result

