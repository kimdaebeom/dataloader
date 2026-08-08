# Release checklist

## Before the first public release

- Verify that the MIT license metadata and `LICENSE` file are included in both artifacts.
- Confirm that `autonomous-dataloader` is still available as the PyPI distribution name.
- Confirm maintainer identity and contact details.
- Review upstream dataset licenses, citation requirements, and trademark terms.

The PyPI distribution name, Python import name, and ROS package name may differ. This project uses `autonomous-dataloader` for distribution and `dataloader` for both Python imports and ROS.

## Build and test

```bash
python3 -m pip install --upgrade build twine
python3 -m unittest discover -s tests -v
python3 -m build
python3 -m twine check dist/*
```

Install the wheel into a clean environment and check all commands:

```bash
python3 -m venv /tmp/dataloader-release-test
source /tmp/dataloader-release-test/bin/activate
python3 -m pip install dist/*.whl
dataloader-convert --help
dataloader-convert-many --help
dataloader-info --help
dataloader-validate --help
```

Build once in a clean ROS 1 workspace and once in a clean ROS 2 workspace. Launch a short converted sequence and verify `/clock`, one PointCloud2 stream, and one IMU stream when available.

## Publish

Upload to TestPyPI first:

```bash
python3 -m twine upload --repository testpypi dist/*
```

Install from TestPyPI in a clean environment, then upload the unchanged artifacts to PyPI:

```bash
python3 -m twine upload dist/*
```

Tag the exact commit used to produce the artifacts. The tag, `setup.py`, `package.xml`, and `dataloader.__version__` must use the same version.
