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
	colcon test --base-paths ros2

	# Run Rust tests
	cargo nextest run --no-fail-fast --cargo-profile release-with-debug

	# Run Python tests for launch2dump
	cd python && \
	. /opt/ros/humble/setup.sh && \
	uv run pytest

.PHONY: format
format:
	cargo +nightly fmt
	cd python && uv run ruff format launch2dump/src/ launch2dump/tests/

.PHONY: lint
lint:
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings
	cd python && uv run ruff check launch2dump/src/ launch2dump/tests/
	cd python && uv run ruff format --check launch2dump/src/ launch2dump/tests/

.PHONY: clean
clean:
	cargo clean
