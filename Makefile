.PHONY: help
help:
	@echo "Available targets:"
	@echo "  build   - Build all targets"
	@echo "  test    - Run all tests (Rust + Python)"
	@echo "  format  - Format code with rustfmt"
	@echo "  lint    - Check formatting and run clippy"
	@echo "  clean   - Clean build artifacts"

.PHONY: build
build:
	# Build Rust workspace
	cargo build --all-targets --profile release-with-debug
	# Build ROS 2 packages
	colcon build --base-paths ros2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: test
test:
	# Run ROS 2 package tests
	. /opt/ros/humble/setup.sh && \
	colcon test --base-paths ros2

	# Run Rust tests
	cargo nextest run --no-fail-fast --cargo-profile release-with-debug

	# Run Python tests (per-package to avoid import conflicts)
	# Automatically discovers all packages in python/ directory
	. install/setup.sh && \
	for pkg in python/*/pyproject.toml; do \
		pkg_dir=$$(dirname $$pkg); \
		echo "Testing $$pkg_dir..."; \
		(cd $$pkg_dir && uv run pytest) || exit 1; \
	done

.PHONY: format
format:
	# Format Rust code
	cargo +nightly fmt
	# Format Python code (automatically discovers all packages)
	for pkg in python/*/pyproject.toml; do \
		pkg_dir=$$(dirname $$pkg); \
		pkg_name=$$(basename $$pkg_dir); \
		echo "Formatting $$pkg_name..."; \
		uv run ruff format $$pkg_dir/src/ $$pkg_dir/tests/ 2>/dev/null || true; \
	done

.PHONY: lint
lint:
	# Lint Rust code
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings
	# Lint Python code (automatically discovers all packages)
	for pkg in python/*/pyproject.toml; do \
		pkg_dir=$$(dirname $$pkg); \
		pkg_name=$$(basename $$pkg_dir); \
		echo "Linting $$pkg_name..."; \
		uv run ruff check $$pkg_dir/src/ $$pkg_dir/tests/ || exit 1; \
		uv run ruff format --check $$pkg_dir/src/ $$pkg_dir/tests/ || exit 1; \
	done

.PHONY: clean
clean:
	rm -rf build install log
	cargo clean
