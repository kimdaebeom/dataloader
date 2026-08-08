"""NumPy readers for point-cloud formats declared in converted manifests."""

from pathlib import Path

import numpy as np

AEVA_INTENSITY_THRESHOLD_NS = 1691936557946849179


class UnsupportedPointCloudFormat(ValueError):
    pass


def _packed_dtype(fields, itemsize):
    return np.dtype(
        {
            "names": [field[0] for field in fields],
            "formats": [field[1] for field in fields],
            "offsets": [field[2] for field in fields],
            "itemsize": itemsize,
        }
    )


_XYZI = _packed_dtype(
    [
        ("x", "<f4", 0),
        ("y", "<f4", 4),
        ("z", "<f4", 8),
        ("intensity", "<f4", 12),
    ],
    16,
)

_DTYPES = {
    "mulran_ouster": _XYZI,
    "semantic_kitti_velodyne": _XYZI,
    "helipr_ouster": _packed_dtype(
        [
            ("x", "<f4", 0),
            ("y", "<f4", 4),
            ("z", "<f4", 8),
            ("intensity", "<f4", 12),
            ("t", "<u4", 16),
            ("reflectivity", "<u2", 20),
            ("ring", "<u2", 22),
            ("ambient", "<u2", 24),
        ],
        26,
    ),
    "helipr_velodyne": _packed_dtype(
        [
            ("x", "<f4", 0),
            ("y", "<f4", 4),
            ("z", "<f4", 8),
            ("intensity", "<f4", 12),
            ("ring", "<u2", 16),
            ("time", "<f4", 18),
        ],
        22,
    ),
    "helipr_livox_avia": _packed_dtype(
        [
            ("x", "<f4", 0),
            ("y", "<f4", 4),
            ("z", "<f4", 8),
            ("reflectivity", "u1", 12),
            ("tag", "u1", 13),
            ("line", "u1", 14),
            ("offset_time", "<u4", 15),
        ],
        19,
    ),
}

_AEVA_WITHOUT_INTENSITY = _packed_dtype(
    [
        ("x", "<f4", 0),
        ("y", "<f4", 4),
        ("z", "<f4", 8),
        ("reflectivity", "<f4", 12),
        ("velocity", "<f4", 16),
        ("time_offset_ns", "<i4", 20),
        ("line_index", "u1", 24),
    ],
    25,
)

_AEVA_WITH_INTENSITY = _packed_dtype(
    [
        ("x", "<f4", 0),
        ("y", "<f4", 4),
        ("z", "<f4", 8),
        ("reflectivity", "<f4", 12),
        ("velocity", "<f4", 16),
        ("time_offset_ns", "<i4", 20),
        ("line_index", "u1", 24),
        ("intensity", "<f4", 25),
    ],
    29,
)


def point_dtype(format_name, timestamp_ns=None):
    """Return the packed NumPy dtype for a manifest point-cloud format."""
    if format_name == "helipr_aeva":
        if timestamp_ns is None:
            raise ValueError("timestamp_ns is required for helipr_aeva")
        if int(timestamp_ns) > AEVA_INTENSITY_THRESHOLD_NS:
            return _AEVA_WITH_INTENSITY
        return _AEVA_WITHOUT_INTENSITY
    try:
        return _DTYPES[format_name]
    except KeyError as exc:
        raise UnsupportedPointCloudFormat(
            "unsupported point-cloud format: {}".format(format_name)
        ) from exc


def point_step(format_name, timestamp_ns=None):
    return point_dtype(format_name, timestamp_ns).itemsize


def read_structured_points(path, format_name, timestamp_ns=None):
    """Read a point cloud without discarding sensor-specific fields."""
    path = Path(path)
    dtype = point_dtype(format_name, timestamp_ns)
    size = path.stat().st_size
    if size % dtype.itemsize:
        raise ValueError(
            "{} size {} is not divisible by point step {} for {}".format(
                path, size, dtype.itemsize, format_name
            )
        )
    return np.fromfile(str(path), dtype=dtype)


def standard_points(points):
    """Convert structured points to a common float32 ``x,y,z,intensity`` array.

    Formats without an explicit intensity field use reflectivity. Sensor-only
    fields remain available from :func:`read_structured_points`.
    """
    names = points.dtype.names or ()
    missing = [name for name in ("x", "y", "z") if name not in names]
    if missing:
        raise ValueError("point cloud has no fields: {}".format(", ".join(missing)))
    intensity_name = "intensity" if "intensity" in names else "reflectivity"
    if intensity_name not in names:
        intensity = np.zeros(len(points), dtype=np.float32)
    else:
        intensity = points[intensity_name].astype(np.float32, copy=False)
    result = np.empty((len(points), 4), dtype=np.float32)
    result[:, 0] = points["x"]
    result[:, 1] = points["y"]
    result[:, 2] = points["z"]
    result[:, 3] = intensity
    return result
