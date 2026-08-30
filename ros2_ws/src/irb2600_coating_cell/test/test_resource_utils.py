from pathlib import Path

import pytest

from irb2600_coating_cell.resource_utils import resolve_resource_uri


def test_resolve_local_resource(tmp_path):
    resource = tmp_path / "mesh.stl"
    assert resolve_resource_uri(str(resource)) == resource.resolve()


def test_reject_malformed_package_uri():
    with pytest.raises(ValueError, match="Invalid package resource URI"):
        resolve_resource_uri("package://missing_relative_path")
