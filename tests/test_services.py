import tempfile
import unittest
from pathlib import Path

from dataloader import Dataset, convert_dataset, convert_many, dataset_info, validate_dataset
from dataloader.converters import get_converter

from helpers import make_mulran_sequence


class ServiceApiTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
