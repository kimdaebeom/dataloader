"""Tools for converting autonomous-driving datasets to a common layout.

The conversion API has no ROS dependency. ROS is only needed when importing
and running :mod:`dataloader.player`.
"""

from .batch import BatchConversionResult, convert_many
from .converter import convert_dataset
from .converters import (
    BaseConverter,
    available_converters as available_datasets,
    register_converter,
)
from .dataset import Dataset, Event, LidarFrame, Pose
from .info import dataset_info
from .validation import ValidationReport, validate_dataset

__all__ = [
    "BatchConversionResult",
    "BaseConverter",
    "Dataset",
    "Event",
    "LidarFrame",
    "Pose",
    "ValidationReport",
    "available_datasets",
    "convert_dataset",
    "convert_many",
    "dataset_info",
    "register_converter",
    "validate_dataset",
]
__version__ = "0.1.1"
