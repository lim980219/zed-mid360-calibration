"""CLI: rosrun zed_mid360_calibration calibrate_bags.py ..."""
import argparse
import glob
import json
from pathlib import Path
import sys
import traceback
import numpy as np
import yaml
from .bag_io import Bag
from .config import DEFAULT, load_config
from .prepare import prepare
from .backend import write_inputs, run_backend
from .report import save_report


def bag_paths(inputs):
    found = []
    for item in inputs:
        matches = sorted(glob.glob(str(Path(item).expanduser())))
        if not matches:
            raise ValueError("Input not found: " + item)
        for match in matches:
            path = Path(match).resolve()
            if path.is_dir():
                found.extend(sorted(path.glob("*.bag")))
            elif path.suffix == ".bag":
                found.append(path)
            else:
                raise ValueError("ROS1 .bag files are required: " + str(path))
    found = list(dict.fromkeys(found))
    if not found:
        raise ValueError("No .bag files found.")
    return found


def main(argv=None):
    parser = argparse.ArgumentParser(description="ROS1 Noetic ZED2/MID-360 offline calibration")
    parser.add_argument("bags", nargs="*", help="Bag files, quoted glob, or directory of .bag files")
    parser.add_argument("--config", type=Path, help="YAML with measured camera_mount")
    parser.add_argument("--output", type=Path, help="New output directory (never overwritten)")
    parser.add_argument("--inspect", action="store_true", help="List bag topics without calibration")
    parser.add_argument("--prepare-only", action="store_true", help="Extract BMP/PCD and backend settings only")
    parser.add_argument("--write-config", type=Path, help="Create a config template and exit")
    args = parser.parse_args(argv)
    output = None
    try:
        if args.write_config:
            with args.write_config.open("x", encoding="utf-8") as stream:
                yaml.safe_dump(DEFAULT, stream, sort_keys=False)
            print("Fill camera_mount.forward_m, left_m, down_m in " + str(args.write_config))
            return 0
        paths = bag_paths(args.bags)
        if args.inspect:
            for path in paths:
                with Bag(path) as bag:
                    print(str(path))
                    for topic, kind in sorted(bag.topics.items()):
                        print("  {}: {}".format(topic, kind))
            return 0
        if args.config is None or args.output is None:
            parser.error("--config and --output are required for calibration")
        cfg = load_config(args.config)
        candidate = args.output.expanduser().resolve()
        candidate.mkdir(parents=True, exist_ok=False)
        output = candidate
        (output / "effective_config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
        if len(paths) < 3:
            print("Note: fewer than 3 scenes; use multiple diverse stationary scenes for a better constrained estimate.")
        scenes = prepare(paths, output, cfg)
        initial = write_inputs(output, scenes, cfg)
        if args.prepare_only:
            print("Prepared {} scenes: {} (optimizer not executed)".format(len(scenes), output))
            return 0
        print("[calibrate] Running HKU multi-scene optimizer...", flush=True)
        result, diagnostics = run_backend(output, cfg)
        report = save_report(output, scenes, cfg, initial, result, diagnostics)
        print("T_camera_lidar: p_camera_optical = T_camera_lidar @ p_lidar")
        print(np.array(report["T_camera_lidar"]))
        print("Status: " + report["status"])
        print("Results: " + str(output / "extrinsics.yaml"))
        print("Review projections: " + str(output / "overlays"))
        return 2 if report["issues"] else 0
    except KeyboardInterrupt:
        print("Cancelled.", file=sys.stderr)
        return 130
    except Exception as error:
        print("ERROR: " + str(error), file=sys.stderr)
        if output is not None:
            (output / "failure.json").write_text(json.dumps(
                {"status": "failed", "error": str(error), "traceback": traceback.format_exc()}, indent=2),
                encoding="utf-8")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

