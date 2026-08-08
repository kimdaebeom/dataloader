"""Registry for raw-dataset converter adapters."""

from dataloader.common import validate_path_component

from .base import BaseConverter
from .helipr import HeLiPRConverter
from .mulran import MulRanConverter
from .semantic_kitti import SemanticKittiConverter

_CONVERTERS = {}


def register_converter(converter, replace=False):
    """Register a converter adapter for CLI and Python conversion."""
    if not isinstance(converter, BaseConverter):
        raise TypeError("converter must be a BaseConverter instance")
    if not converter.name:
        raise ValueError("converter.name must not be empty")
    name = validate_path_component(converter.name, "converter.name")
    if name in _CONVERTERS and not replace:
        raise ValueError("converter '{}' is already registered".format(name))
    _CONVERTERS[name] = converter
    return converter


for _converter in (
    HeLiPRConverter(),
    MulRanConverter(),
    SemanticKittiConverter(),
):
    register_converter(_converter)


def available_converters():
    return tuple(sorted(_CONVERTERS))


def get_converter(name):
    try:
        return _CONVERTERS[name]
    except KeyError as exc:
        raise ValueError(
            "unknown dataset '{}'; expected one of: {}".format(
                name, ", ".join(available_converters())
            )
        ) from exc


__all__ = [
    "BaseConverter",
    "available_converters",
    "get_converter",
    "register_converter",
]
