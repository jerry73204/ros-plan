"""
File registry for tracking multi-file plan generation.

This module provides infrastructure for managing multiple plan output files
with proper path resolution and deduplication.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Set


@dataclass
class PackageInfo:
    """Information about a ROS package containing a launch file."""

    name: str  # Package name
    root_path: Path  # Package root directory
    relative_path: Path  # Path relative to package root


@dataclass
class FileEntry:
    """Entry in the file registry tracking a generated plan file."""

    source_path: Path  # Canonical path to source launch file
    output_path: Path  # Path where plan file will be written
    package_info: Optional[PackageInfo] = None  # Package info if applicable


class FileRegistry:
    """
    Registry for tracking all generated plan files in multi-file conversion.

    Ensures:
    - Each unique launch file generates exactly one plan file
    - Consistent path resolution (package-based or path-based)
    - No path collisions
    - Deduplication by canonical source path
    """

    def __init__(self, output_dir: Path):
        """
        Initialize the file registry.

        Args:
            output_dir: Base directory for all generated plan files
        """
        self.output_dir = output_dir
        self._entries: Dict[Path, FileEntry] = {}  # canonical source path -> entry
        self._output_paths: Set[Path] = set()  # Track used output paths

    def register_file(
        self,
        source_path: Path,
        package_info: Optional[PackageInfo] = None,
    ) -> Path:
        """
        Register a launch file for conversion and get its output path.

        If the file is already registered, returns the existing output path.

        Args:
            source_path: Path to source launch file (will be canonicalized)
            package_info: Package information if file is in a package

        Returns:
            Output path where the plan file will be written

        Raises:
            ValueError: If output path collision detected
        """
        # Canonicalize source path
        canonical_source = source_path.resolve()

        # Check if already registered
        if canonical_source in self._entries:
            return self._entries[canonical_source].output_path

        # Resolve output path
        output_path = self._resolve_output_path(canonical_source, package_info)

        # Check for collisions
        if output_path in self._output_paths:
            raise ValueError(f"Output path collision: {output_path} (source: {canonical_source})")

        # Create and store entry
        entry = FileEntry(
            source_path=canonical_source,
            output_path=output_path,
            package_info=package_info,
        )
        self._entries[canonical_source] = entry
        self._output_paths.add(output_path)

        return output_path

    def _resolve_output_path(
        self,
        source_path: Path,
        package_info: Optional[PackageInfo],
    ) -> Path:
        """
        Resolve output path for a source file using package or path-based strategy.

        Package-based: output_dir/package_name/relative/path.plan.yaml
        Path-based: output_dir/_path/absolute/path.plan.yaml

        Args:
            source_path: Canonical source file path
            package_info: Package information (if applicable)

        Returns:
            Resolved output path
        """
        # Get stem without .launch extension if present
        stem = source_path.stem
        if stem.endswith(".launch"):
            stem = stem[:-7]  # Remove ".launch"

        plan_filename = f"{stem}.plan.yaml"

        if package_info:
            # Package-based path
            output_path = (
                self.output_dir
                / package_info.name
                / package_info.relative_path.parent
                / plan_filename
            )
        else:
            # Path-based for files outside packages
            # Use _path prefix to avoid collisions with package names
            # Remove leading slash from absolute path
            rel_path = str(source_path.parent).lstrip("/")
            output_path = self.output_dir / "_path" / rel_path / plan_filename

        return output_path

    def get_output_path(self, source_path: Path) -> Optional[Path]:
        """
        Get output path for a registered source file.

        Args:
            source_path: Path to source launch file

        Returns:
            Output path if registered, None otherwise
        """
        canonical_source = source_path.resolve()
        entry = self._entries.get(canonical_source)
        return entry.output_path if entry else None

    def get_all_files(self) -> Dict[Path, Path]:
        """
        Get all registered files as a mapping from source to output paths.

        Returns:
            Dictionary mapping source path -> output path
        """
        return {entry.source_path: entry.output_path for entry in self._entries.values()}

    def is_registered(self, source_path: Path) -> bool:
        """
        Check if a source file is already registered.

        Args:
            source_path: Path to source launch file

        Returns:
            True if file is registered, False otherwise
        """
        canonical_source = source_path.resolve()
        return canonical_source in self._entries

    def get_entry(self, source_path: Path) -> Optional[FileEntry]:
        """
        Get the registry entry for a source file.

        Args:
            source_path: Path to source launch file

        Returns:
            FileEntry if registered, None otherwise
        """
        canonical_source = source_path.resolve()
        return self._entries.get(canonical_source)

    def __len__(self) -> int:
        """Get number of registered files."""
        return len(self._entries)
