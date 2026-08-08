import tempfile
import unittest
from pathlib import Path
from subprocess import run
from sys import executable

from helpers import make_mulran_sequence

from dataloader import (
    Dataset,
    convert_dataset,
    convert_many,
    dataset_info,
    validate_dataset,
)
from dataloader.converters import get_converter


class ServiceApiTest(unittest.TestCase):
    def test_core_import_does_not_load_ros_or_opencv(self):
        check = run(
            [
                executable,
                "-c",
                (
                    "import sys; import dataloader; "
                    "blocked = {'cv2', 'rospy', 'rclpy', 'dataloader.player'} & set(sys.modules); "
                    "assert not blocked, blocked"
                ),
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(check.returncode, 0, check.stderr)

    def test_validate_and_info(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source, _ = make_mulran_sequence(root / "raw")
            converted = convert_dataset("mulran", source, root / "converted")

            report = validate_dataset(converted["sequence_dir"])
            self.assertTrue(report.ok, report.issues)
            self.assertEqual(report.stats["events"], 2)

            info = dataset_info(converted["sequence_dir"])
            self.assertEqual(info["dataset"], "mulran")
            self.assertEqual(info["sensors"]["ouster"]["events"], 2)

            first_lidar = next(
                (converted["sequence_dir"] / "sensors" / "lidar" / "ouster").glob("*.bin")
            )
            with first_lidar.open("ab") as handle:
                handle.write(b"x")
            broken = validate_dataset(converted["sequence_dir"])
            self.assertFalse(broken.ok)
            self.assertTrue(
                any(issue.code == "invalid_point_file_size" for issue in broken.errors)
            )

    def test_batch_conversion_and_converter_registry(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            make_mulran_sequence(root / "raw", "A")
            make_mulran_sequence(root / "raw", "B")
            result = convert_many(
                "mulran",
                root / "raw",
                root / "converted",
                workers=2,
            )
            self.assertTrue(result.ok)
            self.assertEqual(result.successful, ["A", "B"])
            self.assertEqual(get_converter("mulran").name, "mulran")

    def test_batch_reports_incomplete_conversion_as_failure(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source, stamps = make_mulran_sequence(root / "raw", "BROKEN")
            (source / "sensor_data" / "Ouster" / "{}.bin".format(stamps[0])).unlink()
            result = convert_many("mulran", root / "raw", root / "converted")
            self.assertFalse(result.ok)
            self.assertIn("BROKEN", result.failed)
            self.assertFalse(result.results["BROKEN"]["ok"])

    def test_validation_supports_reference_and_symlink_storage(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source, _ = make_mulran_sequence(root / "raw")
            for mode in ("reference", "symlink"):
                with self.subTest(mode=mode):
                    converted = convert_dataset(
                        "mulran",
                        source,
                        root / "converted",
                        sequence=mode,
                        link_mode=mode,
                    )
                    report = validate_dataset(converted["sequence_dir"])
                    self.assertTrue(report.ok, report.issues)

    def test_reader_rejects_metadata_path_escape(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source, _ = make_mulran_sequence(root / "raw")
            converted = convert_dataset("mulran", source, root / "converted")
            manifest_path = converted["manifest_path"]
            manifest = manifest_path.read_text()
            manifest_path.write_text(
                manifest.replace("timeline: timeline.csv", "timeline: ../timeline.csv")
            )
            with self.assertRaisesRegex(ValueError, "escapes"):
                Dataset(converted["sequence_dir"])
            report = validate_dataset(converted["sequence_dir"])
            self.assertFalse(report.ok)
            self.assertTrue(any(issue.code == "timeline_path" for issue in report.errors))

    def test_reader_and_validator_reject_unknown_format_version(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source, _ = make_mulran_sequence(root / "raw")
            converted = convert_dataset("mulran", source, root / "converted")
            manifest_path = converted["manifest_path"]
            manifest_path.write_text(
                manifest_path.read_text().replace("format_version: 1", "format_version: 99")
            )
            with self.assertRaisesRegex(ValueError, "unsupported format_version"):
                Dataset(converted["sequence_dir"])
            report = validate_dataset(converted["sequence_dir"])
            self.assertFalse(report.ok)
            self.assertTrue(
                any(
                    issue.code == "unsupported_format_version"
                    for issue in report.errors
                )
            )

    def test_raw_deletion_rejects_partial_or_non_copy_conversion(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            source_root = Path(temporary_directory) / "raw"
            source_root.mkdir()
            script = Path(__file__).parents[1] / "scripts" / "convert_sequences.sh"
            common = [
                str(script),
                "--dataset",
                "mulran",
                "--source-root",
                str(source_root),
                "--output",
                str(Path(temporary_directory) / "converted"),
                "--delete-source-after-success",
            ]
            partial = run(
                common + ["--start-lidar-frame", "1"],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(partial.returncode, 0)
            self.assertIn("cannot be combined", partial.stderr)

            linked = run(
                common + ["--link-mode", "symlink"],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(linked.returncode, 0)
            self.assertIn("requires --link-mode copy", linked.stderr)


if __name__ == "__main__":
    unittest.main()
