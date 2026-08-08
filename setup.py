#!/usr/bin/env python3

"""Compatibility entry point for catkin and legacy pip versions."""

from pathlib import Path

from setuptools import find_packages, setup


ROOT = Path(__file__).parent


setup_arguments = dict(
    name="autodataloader",
    version="0.1.0",
    description="Convert autonomous-driving datasets into a unified, timestamped layout.",
    long_description=(ROOT / "README.md").read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    python_requires=">=3.8",
    author="Kim Daebeom",
    author_email="kimdaebeom@users.noreply.github.com",
    license="MIT",
    license_files=["LICENSE"],
    url="https://github.com/kimdaebeom/dataloader",
    project_urls={
        "Documentation": "https://github.com/kimdaebeom/dataloader/tree/master/docs",
        "Issues": "https://github.com/kimdaebeom/dataloader/issues",
        "Source": "https://github.com/kimdaebeom/dataloader",
    },
    keywords="autonomous-driving lidar robotics dataset converter",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering",
    ],
    package_dir={"": "src"},
    packages=find_packages("src"),
    install_requires=[
        "numpy>=1.20",
        "PyYAML>=5.4",
    ],
    extras_require={"dev": ["build>=0.10", "twine>=4"]},
)

# catkin replaces setuptools.setup() while inspecting this file and does not
# support entry_points in its devel space. The existing catkin-installed script
# remains available there; normal setuptools/pip installs get the ROS-free CLI.
if setup.__module__ == "setuptools":
    setup_arguments["entry_points"] = {
        "console_scripts": [
            "dataloader-convert=dataloader.converter:main",
            "dataloader-convert-many=dataloader.batch:main",
            "dataloader-info=dataloader.info:main",
            "dataloader-validate=dataloader.validation:main",
        ],
    }

setup(**setup_arguments)
