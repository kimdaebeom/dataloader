# Release checklist

<a id="before-the-first-public-release"></a>
<details open>
<summary><strong>Before the first public release</strong></summary>

<br>

- Verify that the MIT license metadata and `LICENSE` file are included in both artifacts.
- Confirm that `autonomous-dataloader` is still available as the PyPI distribution name.
- Confirm maintainer identity and contact details.
- Review upstream dataset licenses, citation requirements, and trademark terms.

The PyPI distribution name, Python import name, and ROS package name may differ. This project uses `autonomous-dataloader` for distribution and `dataloader` for both Python imports and ROS.

</details>

<a id="build-and-test"></a>
<details>
<summary><strong>Build and test</strong></summary>

<br>

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

</details>

<a id="configure-pypi-trusted-publishing"></a>
<details>
<summary><strong>Configure PyPI Trusted Publishing</strong></summary>

<br>

The repository publishes through `.github/workflows/publish-to-pypi.yml` without a long-lived API token. Before the first release, create a pending GitHub publisher at <https://pypi.org/manage/account/publishing/> with:

| Field | Value |
| --- | --- |
| PyPI project name | `autonomous-dataloader` |
| GitHub owner | `kimdaebeom` |
| Repository | `dataloader` |
| Workflow | `publish-to-pypi.yml` |
| Environment | `pypi` |

Create the `pypi` environment in the GitHub repository and require manual approval for deployment.

</details>

<a id="publish"></a>
<details>
<summary><strong>Publish</strong></summary>

<br>

1. Verify that the release commit passes the Python and ROS workflows.
2. Tag the exact release commit, for example `v0.1.0`, and push the tag.
3. Approve the `pypi` environment deployment when prompted.
4. If a tag run must be retried, run `Publish Python package` manually for the same commit.
5. Verify the public install in a clean environment:

```bash
python3 -m venv /tmp/dataloader-pypi-test
source /tmp/dataloader-pypi-test/bin/activate
python3 -m pip install autonomous-dataloader
python3 -c "import dataloader; print(dataloader.__version__)"
```

The GitHub tag, `setup.py`, `package.xml`, and `dataloader.__version__` must use the same version.

</details>
