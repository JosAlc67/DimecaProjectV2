from pathlib import Path

import pytest
import trimesh


_MESH_DIR = Path(__file__).parents[1] / "meshes" / "cad"


@pytest.mark.parametrize(
    ("name", "expected_extents"),
    [
        ("cobra_final.stl", (0.342479, 0.315413, 0.108000)),
        ("motioncam_housing.stl", (0.092000, 0.665000, 0.088000)),
    ],
)
def test_final_cad_meshes_are_meter_scale(name, expected_extents):
    mesh = trimesh.load_mesh(_MESH_DIR / name, process=False)
    assert tuple(mesh.extents) == pytest.approx(expected_extents, abs=1e-5)
