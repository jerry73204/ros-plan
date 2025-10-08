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
	cargo build --all-targets --profile release-with-debug

.PHONY: test
test:
# Run Rust tests
	cargo nextest run --no-fail-fast --cargo-profile release-with-debug

# Run Python tests
	cd launch2dump && \
	. /opt/ros/humble/setup.sh && \
	uv run pytest

.PHONY: format
format:
	cargo +nightly fmt

.PHONY: lint
lint:
	cargo +nightly fmt --check
	cargo clippy --all-targets --all-features -- -D warnings

.PHONY: clean
clean:
	cargo clean
