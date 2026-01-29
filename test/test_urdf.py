from __future__ import annotations

from pathlib import Path
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
from urdf_parser_py.urdf import URDF


def _iter_urdf_files(pkg_share: Path) -> list[Path]:
    urdf_root = pkg_share / "urdf"
    return sorted(urdf_root.rglob("*.urdf"))


def _resolve_mesh_filename(filename: str, urdf_path: Path) -> Path | None:
    # Common patterns: package://pkg_name/path/to/mesh, relative paths, absolute paths.
    if not filename:
        return None
    if filename.startswith("package://"):
        remainder = filename[len("package://") :]
        pkg, _, rel = remainder.partition("/")
        share = Path(get_package_share_directory(pkg))
        return share / rel
    if filename.startswith("file://"):
        return Path(filename[len("file://") :])
    path = Path(filename)
    if path.is_absolute():
        return path
    return (urdf_path.parent / path).resolve()


def _mesh_filenames(xml_root: ET.Element) -> list[str]:
    filenames: list[str] = []
    for mesh in xml_root.findall(".//mesh"):
        filename = mesh.attrib.get("filename") or mesh.attrib.get("url") or ""
        filename = filename.strip()
        if filename:
            filenames.append(filename)
    return filenames


@pytest.mark.parametrize("urdf_path", _iter_urdf_files(Path(get_package_share_directory("mycobot320pi_urdf"))))
def test_urdf_parses_and_has_expected_links(urdf_path: Path) -> None:
    xml_text = urdf_path.read_text(encoding="utf-8")
    urdf = URDF.from_xml_string(xml_text)

    link_names = {link.name for link in urdf.links}
    assert "base" in link_names
    assert "link6" in link_names
    assert "tcp_link" not in link_names

    # Ensure no joints reference tcp_link anywhere.
    assert re.search(r"\\btcp_link\\b", xml_text) is None


@pytest.mark.parametrize("urdf_path", _iter_urdf_files(Path(get_package_share_directory("mycobot320pi_urdf"))))
def test_mesh_references_exist(urdf_path: Path) -> None:
    xml_root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    for filename in _mesh_filenames(xml_root):
        resolved = _resolve_mesh_filename(filename, urdf_path)
        assert resolved is not None
        assert resolved.exists(), f"Missing mesh referenced by URDF: {filename} -> {resolved}"

