import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from dataloader import Dataset, convert_dataset
from dataloader.lidar import read_structured_points, standard_points

from helpers import make_mulran_sequence


class DatasetApiTest(unittest.TestCase):
    def test_reader_lidar_and_time_queries(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source, stamps = make_mulran_sequence(root / "raw")
            result = convert_dataset("mulran", source, root / "converted")
            dataset = Dataset(result["sequence_dir"])

            self.assertEqual(len(dataset), 2)
            self.assertEqual(dataset.duration_ns, stamps[1] - stamps[0])
            self.assertEqual(dataset.nearest("ouster", stamps[0] + 1).timestamp_ns, stamps[0])
            self.assertEqual(
                len(dataset.between("ouster", stamps[0], stamps[0])),
                1,
            )

            frame = dataset.lidar(frame=0)
            points = frame.numpy()
            structured = frame.structured()
            self.assertEqual(points.shape, (2, 4))
            np.testing.assert_allclose(points[0], [1.0, 2.0, 3.0, 4.0])
            self.assertEqual(structured.dtype.names, ("x", "y", "z", "intensity"))

    def test_sensor_specific_lidar_formats_preserve_extra_fields(self):
        cases = {
            "helipr_ouster": (
                1,
                struct.pack("<ffffIHHH", 1.0, 2.0, 3.0, 4.0, 5, 6, 7, 8),
                "ring",
            ),
            "helipr_velodyne": (
                1,
                struct.pack("<ffffHf", 1.0, 2.0, 3.0, 4.0, 5, 0.1),
                "time",
            ),
            "helipr_livox_avia": (
                1,
                struct.pack("<fffBBBI", 1.0, 2.0, 3.0, 4, 5, 6, 7),
                "offset_time",
            ),
            "helipr_aeva": (
                1,
                struct.pack("<fffffiB", 1.0, 2.0, 3.0, 4.0, 5.0, 6, 7),
                "velocity",
            ),
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for format_name, (stamp, payload, extra_field) in cases.items():
                path = root / "{}.bin".format(format_name)
                path.write_bytes(payload)
                structured = read_structured_points(path, format_name, stamp)
                self.assertIn(extra_field, structured.dtype.names)
                common = standard_points(structured)
                self.assertEqual(common.shape, (1, 4))
                self.assertEqual(common[0, 3], 4.0)


if __name__ == "__main__":
    unittest.main()
