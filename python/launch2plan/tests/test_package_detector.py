"""Tests for package_detector module."""

from pathlib import Path

from launch2plan.package_detector import (
    detect_package,
    get_package_name,
    get_package_root,
    is_in_package,
)


class TestDetectPackage:
    """Test detect_package function."""

    def test_detect_package_in_package(self, tmp_path):
        """Test detecting a file inside a package."""
        # Create package structure
        pkg_root = tmp_path / "my_package"
        pkg_root.mkdir()

        # Create package.xml
        package_xml = pkg_root / "package.xml"
        package_xml.write_text("""<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>1.0.0</version>
  <description>Test package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        # Create launch file
        launch_dir = pkg_root / "launch"
        launch_dir.mkdir()
        launch_file = launch_dir / "test.launch.py"
        launch_file.touch()

        # Detect package
        pkg_info = detect_package(launch_file)

        assert pkg_info is not None
        assert pkg_info.name == "my_package"
        assert pkg_info.root_path == pkg_root
        assert pkg_info.relative_path == Path("launch/test.launch.py")

    def test_detect_package_nested_directory(self, tmp_path):
        """Test detecting package for file in nested directory."""
        # Create package structure
        pkg_root = tmp_path / "nav_package"
        pkg_root.mkdir()

        # Create package.xml
        package_xml = pkg_root / "package.xml"
        package_xml.write_text("""<?xml version="1.0"?>
<package format="3">
  <name>nav_package</name>
  <version>1.0.0</version>
  <description>Navigation package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        # Create deeply nested launch file
        launch_file = pkg_root / "launch" / "config" / "params" / "navigation.launch.py"
        launch_file.parent.mkdir(parents=True)
        launch_file.touch()

        # Detect package
        pkg_info = detect_package(launch_file)

        assert pkg_info is not None
        assert pkg_info.name == "nav_package"
        assert pkg_info.root_path == pkg_root
        assert pkg_info.relative_path == Path("launch/config/params/navigation.launch.py")

    def test_detect_package_file_outside_package(self, tmp_path):
        """Test detecting file outside any package."""
        # Create file without package.xml in any parent
        launch_file = tmp_path / "standalone" / "robot.launch.py"
        launch_file.parent.mkdir()
        launch_file.touch()

        # Detect package
        pkg_info = detect_package(launch_file)

        assert pkg_info is None

    def test_detect_package_multiple_packages_uses_nearest(self, tmp_path):
        """Test that nearest package.xml is used when multiple exist."""
        # Create outer package
        outer_pkg = tmp_path / "outer_package"
        outer_pkg.mkdir()
        (outer_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>outer_package</name>
  <version>1.0.0</version>
  <description>Outer package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        # Create inner package
        inner_pkg = outer_pkg / "inner_package"
        inner_pkg.mkdir()
        (inner_pkg / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>inner_package</name>
  <version>1.0.0</version>
  <description>Inner package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        # Create file in inner package
        launch_file = inner_pkg / "launch" / "test.launch.py"
        launch_file.parent.mkdir()
        launch_file.touch()

        # Detect package - should find inner package
        pkg_info = detect_package(launch_file)

        assert pkg_info is not None
        assert pkg_info.name == "inner_package"
        assert pkg_info.root_path == inner_pkg

    def test_detect_package_invalid_xml(self, tmp_path):
        """Test handling of invalid package.xml."""
        # Create package with invalid XML
        pkg_root = tmp_path / "bad_package"
        pkg_root.mkdir()

        # Create invalid package.xml
        package_xml = pkg_root / "package.xml"
        package_xml.write_text("This is not valid XML")

        # Create launch file
        launch_file = pkg_root / "launch" / "test.launch.py"
        launch_file.parent.mkdir()
        launch_file.touch()

        # Detect package - should return None due to parse error
        pkg_info = detect_package(launch_file)

        # With invalid XML, it should continue searching up and find nothing
        assert pkg_info is None

    def test_detect_package_missing_name_element(self, tmp_path):
        """Test handling of package.xml without <name> element."""
        # Create package with XML missing <name>
        pkg_root = tmp_path / "incomplete_package"
        pkg_root.mkdir()

        # Create package.xml without <name>
        package_xml = pkg_root / "package.xml"
        package_xml.write_text("""<?xml version="1.0"?>
<package format="3">
  <version>1.0.0</version>
  <description>Package without name</description>
</package>
""")

        # Create launch file
        launch_file = pkg_root / "launch" / "test.launch.py"
        launch_file.parent.mkdir()
        launch_file.touch()

        # Detect package - should return None
        pkg_info = detect_package(launch_file)

        assert pkg_info is None

    def test_detect_package_preserves_path_structure(self, tmp_path):
        """Test that relative path preserves directory structure."""
        # Create package
        pkg_root = tmp_path / "sensor_package"
        pkg_root.mkdir()

        (pkg_root / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>sensor_package</name>
  <version>1.0.0</version>
  <description>Sensor package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        # Create launch file with specific structure
        launch_file = pkg_root / "launch" / "sensors" / "camera.launch.py"
        launch_file.parent.mkdir(parents=True)
        launch_file.touch()

        # Detect package
        pkg_info = detect_package(launch_file)

        assert pkg_info is not None
        assert pkg_info.relative_path == Path("launch/sensors/camera.launch.py")


class TestIsInPackage:
    """Test is_in_package function."""

    def test_is_in_package_true(self, tmp_path):
        """Test file inside package."""
        pkg_root = tmp_path / "test_pkg"
        pkg_root.mkdir()

        (pkg_root / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>1.0.0</version>
  <description>Test</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        launch_file = pkg_root / "launch" / "test.launch.py"
        launch_file.parent.mkdir()
        launch_file.touch()

        assert is_in_package(launch_file) is True

    def test_is_in_package_false(self, tmp_path):
        """Test file outside package."""
        launch_file = tmp_path / "test.launch.py"
        launch_file.touch()

        assert is_in_package(launch_file) is False


class TestGetPackageName:
    """Test get_package_name function."""

    def test_get_package_name_in_package(self, tmp_path):
        """Test getting package name for file in package."""
        pkg_root = tmp_path / "my_pkg"
        pkg_root.mkdir()

        (pkg_root / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>my_pkg</name>
  <version>1.0.0</version>
  <description>Test</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        launch_file = pkg_root / "launch" / "test.launch.py"
        launch_file.parent.mkdir()
        launch_file.touch()

        assert get_package_name(launch_file) == "my_pkg"

    def test_get_package_name_outside_package(self, tmp_path):
        """Test getting package name for file outside package."""
        launch_file = tmp_path / "test.launch.py"
        launch_file.touch()

        assert get_package_name(launch_file) is None


class TestGetPackageRoot:
    """Test get_package_root function."""

    def test_get_package_root_in_package(self, tmp_path):
        """Test getting package root for file in package."""
        pkg_root = tmp_path / "root_pkg"
        pkg_root.mkdir()

        (pkg_root / "package.xml").write_text("""<?xml version="1.0"?>
<package format="3">
  <name>root_pkg</name>
  <version>1.0.0</version>
  <description>Test</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>Apache-2.0</license>
</package>
""")

        launch_file = pkg_root / "launch" / "sub" / "test.launch.py"
        launch_file.parent.mkdir(parents=True)
        launch_file.touch()

        assert get_package_root(launch_file) == pkg_root

    def test_get_package_root_outside_package(self, tmp_path):
        """Test getting package root for file outside package."""
        launch_file = tmp_path / "test.launch.py"
        launch_file.touch()

        assert get_package_root(launch_file) is None
