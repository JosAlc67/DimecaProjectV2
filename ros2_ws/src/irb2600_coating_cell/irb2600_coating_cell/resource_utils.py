"""Portable helpers for package resources and shared cell configuration."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def resolve_resource_uri(uri: str) -> Path:
    """Resolve a package:// URI without machine-specific fallback paths."""
    if not uri.startswith("package://"):
        return Path(uri).expanduser().resolve()

    remainder = uri[len("package://") :]
    package_name, separator, relative_path = remainder.partition("/")
    if not package_name or not separator or not relative_path:
        raise ValueError(f"Invalid package resource URI: {uri}")
    return Path(get_package_share_directory(package_name), relative_path)


def load_shared_cell_config(filename: str = "scene_objects.yaml") -> dict:
    """Load the wildcard ROS parameters from the package's shared YAML."""
    config_path = Path(
        get_package_share_directory("irb2600_coating_cell"), "config", filename
    )
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    return document.get("/**", {}).get("ros__parameters", {})
