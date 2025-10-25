"""
Package detection for ROS 2 launch files.

This module provides utilities to detect if a launch file belongs to a ROS package
and extract package metadata.
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

from .file_registry import PackageInfo


def detect_package(file_path: Path) -> Optional[PackageInfo]:
    """
    Detect if a launch file belongs to a ROS package.

    Searches parent directories for package.xml and extracts package metadata.

    Args:
        file_path: Path to the launch file

    Returns:
        PackageInfo if file is in a package, None otherwise
    """
    # Resolve to absolute path
    file_path = file_path.resolve()

    # Search up the directory tree for package.xml
    current_dir = file_path.parent
    while current_dir != current_dir.parent:  # Stop at filesystem root
        package_xml = current_dir / "package.xml"
        if package_xml.exists():
            # Found package.xml - extract package info
            try:
                package_name = _extract_package_name(package_xml)
                if package_name:
                    # Calculate relative path from package root to launch file
                    try:
                        relative_path = file_path.relative_to(current_dir)
                    except ValueError:
                        # File is not actually under this directory (shouldn't happen)
                        return None

                    return PackageInfo(
                        name=package_name,
                        root_path=current_dir,
                        relative_path=relative_path,
                    )
            except Exception:
                # Failed to parse package.xml - continue searching
                pass

        # Move up one directory
        current_dir = current_dir.parent

    # No package.xml found
    return None


def _extract_package_name(package_xml_path: Path) -> Optional[str]:
    """
    Extract package name from package.xml.

    Args:
        package_xml_path: Path to package.xml file

    Returns:
        Package name if found, None otherwise
    """
    try:
        tree = ET.parse(package_xml_path)
        root = tree.getroot()

        # Find <name> element
        name_elem = root.find("name")
        if name_elem is not None and name_elem.text:
            return name_elem.text.strip()

        return None
    except (ET.ParseError, FileNotFoundError):
        return None


def is_in_package(file_path: Path) -> bool:
    """
    Check if a file is inside a ROS package.

    Args:
        file_path: Path to check

    Returns:
        True if file is in a package, False otherwise
    """
    return detect_package(file_path) is not None


def get_package_name(file_path: Path) -> Optional[str]:
    """
    Get the package name for a file.

    Args:
        file_path: Path to check

    Returns:
        Package name if file is in a package, None otherwise
    """
    pkg_info = detect_package(file_path)
    return pkg_info.name if pkg_info else None


def get_package_root(file_path: Path) -> Optional[Path]:
    """
    Get the package root directory for a file.

    Args:
        file_path: Path to check

    Returns:
        Package root directory if file is in a package, None otherwise
    """
    pkg_info = detect_package(file_path)
    return pkg_info.root_path if pkg_info else None
