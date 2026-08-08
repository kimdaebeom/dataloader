"""Base interface for raw-dataset converters."""

from abc import ABC, abstractmethod

from dataloader.common import dataset_definition


class BaseConverter(ABC):
    """Dataset-specific timeline and pose conversion adapter."""

    name = None

    @property
    def definition(self):
        return dataset_definition(self.name)

    @abstractmethod
    def read_timeline(self, source):
        """Return normalized event dictionaries for one raw sequence."""

    def check_source(self, source):
        """Raise a useful error when required source metadata is missing."""
        return None

    def convert_poses(self, source, sequence_dir, present_sensors):
        """Convert available poses and return the manifest pose mapping."""
        return {}
