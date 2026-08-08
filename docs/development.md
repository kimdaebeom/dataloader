# Development

## Local checks

```bash
python3 -m pip install -e .
python3 -m unittest discover -s tests -v
python3 -m compileall -q src scripts launch
bash -n scripts/convert_sequences.sh
python3 -m build
python3 -m twine check dist/*
```

Use a temporary virtual environment to verify that the wheel does not rely on files from the source checkout.

## Project structure

```text
src/dataloader/
├── converters/       # raw-dataset adapters and registry
├── converter.py      # shared conversion pipeline and CLI
├── dataset.py        # ROS-independent reader
├── lidar.py          # packed LiDAR format definitions
├── validation.py     # structural and binary validation
├── player.py         # shared ROS playback implementation
└── ros_compat.py     # ROS 1/ROS 2 runtime bridge
```

`package.xml` format 3 and conditional CMake branches keep catkin and ament builds in one package. Core Python modules must remain importable without a ROS installation.

## Add a dataset converter

Implement `BaseConverter` and register one instance:

```python
from dataloader import BaseConverter, register_converter


class MyConverter(BaseConverter):
    name = "my_dataset"

    @property
    def definition(self):
        return {"primary_lidar": "lidar", "sensors": {...}}

    def read_timeline(self, source):
        return [{"timestamp_ns": 0, "sensor": "lidar"}]


register_converter(MyConverter())
```

The adapter should describe dataset-specific timeline, file, and pose rules. Shared path validation, storage modes, manifest creation, and summaries belong in the common pipeline.

## Add a LiDAR format

Define an explicit packed NumPy dtype in `dataloader.lidar` and map it to the manifest format name. The reader and validator must share the same definition; never infer a record layout from a filename alone.

Add tests with a minimal packed binary record that verify both sensor-specific fields and the common `[x, y, z, intensity]` view.

## Compatibility policy

- Keep conversion and reading independent of ROS imports.
- Test ROS 1 and ROS 2 package metadata whenever dependencies change.
- Keep versions synchronized across `setup.py`, `package.xml`, and `dataloader.__version__`.
- Preserve backward compatibility for the converted format or increment `format_version` with a documented migration.
