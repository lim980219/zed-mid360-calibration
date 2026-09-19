"""Native ROS 1 reader; optional rosbags adapter enables offline tests on Windows."""
from pathlib import Path
import numpy as np
import cv2


class Bag:
    def __init__(self, path):
        self.path = Path(path)

    def __enter__(self):
        try:
            import rosbag
        except ImportError:
            from rosbags.highlevel import AnyReader
            self.native = False
            self.reader = AnyReader([self.path])
            self.reader.open()
            self.topics = {c.topic: c.msgtype.replace("/msg/", "/") for c in self.reader.connections}
            self.start = self.reader.start_time / 1e9
            self.end = self.reader.end_time / 1e9
        else:
            self.native = True
            self.reader = rosbag.Bag(str(self.path), "r")
            self.topics = {k: v.msg_type for k, v in self.reader.get_type_and_topic_info().topics.items()}
            self.start = self.reader.get_start_time()
            self.end = self.reader.get_end_time()
        return self

    def __exit__(self, *args):
        self.reader.close()

    def messages(self, topics, start=None, end=None):
        if not topics:
            return
        if self.native:
            from genpy import Time
            for topic, msg, stamp in self.reader.read_messages(
                topics=list(topics),
                start_time=None if start is None else Time.from_sec(start),
                end_time=None if end is None else Time.from_sec(end),
            ):
                yield topic, msg, stamp.to_sec()
        else:
            connections = [c for c in self.reader.connections if c.topic in topics]
            if not connections:
                return
            for c, t, raw in self.reader.messages(
                connections=connections,
                start=None if start is None else int(start * 1e9),
                stop=None if end is None else int(end * 1e9),
            ):
                yield c.topic, self.reader.deserialize(raw, c.msgtype), t / 1e9


def stamp(msg):
    value = msg.header.stamp
    if hasattr(value, "to_sec"):
        result = value.to_sec()
    else:
        result = value.sec + value.nanosec / 1e9
    if not np.isfinite(result) or result <= 0:
        raise ValueError("Non-positive header stamp: synchronize sensor clocks before recording.")
    return result


def matrix_attr(msg, name):
    return np.asarray(getattr(msg, name, getattr(msg, name.lower(), None)), dtype=float)


def decode_image(msg):
    if hasattr(msg, "format"):
        img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("Cannot decode compressed image.")
        return img
    channels = {"mono8": 1, "8UC1": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}
    if msg.encoding not in channels:
        raise ValueError("Unsupported image encoding: " + msg.encoding)
    count = channels[msg.encoding]
    if msg.step < msg.width * count or len(msg.data) < msg.height * msg.step:
        raise ValueError("Truncated Image buffer.")
    rows = np.frombuffer(msg.data, dtype=np.uint8, count=msg.height * msg.step).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * count].reshape(msg.height, msg.width, count).copy()
    if count == 1:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    conversions = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "bgra8": cv2.COLOR_BGRA2BGR}
    return cv2.cvtColor(img, conversions[msg.encoding]) if msg.encoding in conversions else img


def decode_points(msg):
    if hasattr(msg, "point_num"):
        # ROS1 bags embed CustomMsg/CustomPoint definitions; no driver import needed.
        if msg.point_num != len(msg.points):
            raise ValueError("CustomMsg point_num does not match points.")
        return np.asarray([(p.x, p.y, p.z, p.reflectivity) for p in msg.points], dtype=np.float32).reshape(-1, 4)
    fields = {f.name: f for f in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 must contain x, y, z.")
    codes = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    names, formats, offsets = [], [], []
    for name in ("x", "y", "z", "intensity", "reflectivity"):
        if name not in fields:
            continue
        field = fields[name]
        if field.count != 1 or field.datatype not in codes:
            raise ValueError("Unsupported PointCloud2 field: " + name)
        fmt = np.dtype((">" if msg.is_bigendian else "<") + codes[field.datatype])
        if field.offset < 0 or field.offset + fmt.itemsize > msg.point_step:
            raise ValueError("PointCloud2 field lies outside point_step.")
        names.append(name)
        formats.append(fmt)
        offsets.append(field.offset)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError("Truncated PointCloud2 buffer.")
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=msg.point_step))
    array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    intensity = next((array[n].ravel() for n in ("intensity", "reflectivity") if n in names),
                     np.zeros(msg.height * msg.width))
    return np.column_stack([array[n].ravel() for n in ("x", "y", "z")] + [intensity]).astype(np.float32)


def write_pcd(path, points):
    # Binary XYZI PCD understood by pcl::io::loadPCDFile.
    data = np.asarray(points, dtype="<f4")
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
              "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
              "WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              "POINTS {n}\nDATA binary\n").format(n=len(data))
    with Path(path).open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(data.tobytes())

