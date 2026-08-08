import csv
import io
import struct
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import yaml

from dataloader import available_datasets, convert_dataset
from dataloader.converter import main as converter_main
from dataloader.converters import get_converter


class ConverterApiTest(unittest.TestCase):
    def test_available_datasets(self):
        self.assertEqual(
            available_datasets(),
            ("helipr", "mulran", "semantic_kitti"),
        )

    def test_convert_minimal_mulran_sequence(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "raw" / "DCC01"
            output = root / "converted"
            (source / "sensor_data" / "Ouster").mkdir(parents=True)

            stamp = 1_500_000_000_000_000_000
            (source / "sensor_data" / "Ouster" / f"{stamp}.bin").write_bytes(b"lidar")
            with (source / "sensor_data" / "data_stamp.csv").open("w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow([stamp, "ouster"])

            result = convert_dataset(
                dataset="mulran",
                source=source,
                output_root=output,
            )

            self.assertEqual(result["events"], 1)
            self.assertTrue(result["ok"])
            self.assertEqual(result["primary_lidar_frames"], 1)
            self.assertEqual(result["sensors"], ["ouster"])
            self.assertTrue(result["manifest_path"].is_file())

            manifest = yaml.safe_load(result["manifest_path"].read_text())
            self.assertEqual(manifest["dataset"], "mulran")
            self.assertEqual(manifest["sequence"], "DCC01")
            converted_lidar = result["sequence_dir"] / "sensors" / "lidar" / "ouster" / f"{stamp}.bin"
            self.assertEqual(converted_lidar.read_bytes(), b"lidar")

    def test_invalid_link_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown link_mode"):
            convert_dataset(
                dataset="mulran",
                source="/unused",
                output_root="/unused",
                link_mode="invalid",
            )

    def test_sequence_must_not_escape_output_root(self):
        with self.assertRaisesRegex(ValueError, "one non-empty path component"):
            convert_dataset(
                dataset="mulran",
                source="/unused",
                output_root="/unused",
                sequence="../outside",
            )

    def test_source_and_output_must_not_overlap(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "mulran" / "sequence"
            (source / "sensor_data").mkdir(parents=True)
            with self.assertRaisesRegex(ValueError, "must not overlap"):
                convert_dataset(
                    dataset="mulran",
                    source=source,
                    output_root=root,
                    sequence="sequence",
                    overwrite=True,
                )

    def test_empty_timeline_is_rejected_without_partial_output(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "raw" / "EMPTY"
            (source / "sensor_data").mkdir(parents=True)
            (source / "sensor_data" / "data_stamp.csv").write_text("")
            with self.assertRaisesRegex(ValueError, "no supported events"):
                convert_dataset("mulran", source, root / "converted")
            self.assertFalse((root / "converted" / "mulran" / "EMPTY").exists())

    def test_missing_primary_lidar_marks_conversion_incomplete(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "raw" / "MISSING"
            (source / "sensor_data" / "Ouster").mkdir(parents=True)
            stamp = 1_500_000_000_000_000_000
            (source / "sensor_data" / "data_stamp.csv").write_text(
                "{},ouster\n".format(stamp)
            )
            result = convert_dataset("mulran", source, root / "converted")
            self.assertFalse(result["ok"])
            self.assertEqual(result["primary_lidar_frames"], 0)
            self.assertEqual(len(result["missing_files"]), 1)

            cli_output = io.StringIO()
            with redirect_stdout(cli_output):
                exit_code = converter_main(
                    [
                        "--dataset",
                        "mulran",
                        "--source",
                        str(source),
                        "--output",
                        str(root / "cli-converted"),
                    ]
                )
            self.assertEqual(exit_code, 2)
            self.assertIn("status               : INCOMPLETE", cli_output.getvalue())

    def test_failed_overwrite_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "raw" / "DCC01"
            (source / "sensor_data" / "Ouster").mkdir(parents=True)
            stamp = 1_500_000_000_000_000_000
            (source / "sensor_data" / "Ouster" / "{}.bin".format(stamp)).write_bytes(
                b"original"
            )
            (source / "sensor_data" / "data_stamp.csv").write_text(
                "{},ouster\n".format(stamp)
            )
            converted = convert_dataset("mulran", source, root / "converted")
            manifest_before = converted["manifest_path"].read_bytes()

            adapter = get_converter("mulran")
            with mock.patch.object(
                adapter, "convert_poses", side_effect=RuntimeError("pose failure")
            ):
                with self.assertRaisesRegex(RuntimeError, "pose failure"):
                    convert_dataset(
                        "mulran", source, root / "converted", overwrite=True
                    )

            self.assertEqual(converted["manifest_path"].read_bytes(), manifest_before)
            self.assertFalse(
                any(converted["sequence_dir"].parent.glob(".DCC01.convert-*"))
            )

    def test_semantic_kitti_adapter_converts_timeline_and_pose(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "raw" / "00"
            (source / "pcd").mkdir(parents=True)
            (source / "pcd" / "000000.bin").write_bytes(
                struct.pack("<ffff", 1.0, 2.0, 3.0, 4.0)
            )
            (source / "odom_tum.txt").write_text(
                "1000000000 0 0 0 0 0 0 1\n"
            )

            result = convert_dataset("semantic_kitti", source, root / "converted")
            manifest = yaml.safe_load(result["manifest_path"].read_text())
            self.assertEqual(result["events"], 1)
            self.assertEqual(manifest["poses"]["gt"]["velodyne"], "poses/gt.txt")

    def test_helipr_adapter_converts_sensor_specific_pose(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "raw" / "DCC01"
            (source / "LiDAR" / "Ouster").mkdir(parents=True)
            (source / "LiDAR_GT").mkdir()
            stamp = 1_700_000_000_000_000_000
            (source / "LiDAR" / "Ouster" / "{}.bin".format(stamp)).write_bytes(
                struct.pack("<ffffIHHH", 1.0, 2.0, 3.0, 4.0, 5, 6, 7, 8)
            )
            (source / "stamp.csv").write_text("{},ouster\n".format(stamp))
            (source / "LiDAR_GT" / "Ouster_gt.txt").write_text(
                "{} 0 0 0 0 0 0 1\n".format(stamp)
            )

            result = convert_dataset("helipr", source, root / "converted")
            manifest = yaml.safe_load(result["manifest_path"].read_text())
            self.assertEqual(result["events"], 1)
            self.assertEqual(
                manifest["poses"]["gt"]["ouster"],
                "poses/gt_ouster.txt",
            )


if __name__ == "__main__":
    unittest.main()
