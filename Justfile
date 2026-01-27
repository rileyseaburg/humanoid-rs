# Justfile for humanoid-rs

# Default recipe
default:
    @just --list

# Build the project
build:
    cargo build --all-features

# Build release version  
build-release:
    cargo build --release --all-features

# Run tests
test:
    cargo test --all-features

# Run the simulator
sim:
    cargo run --bin humanoid-sim --features simulator

# Run the simulator in release mode
sim-release:
    cargo run --bin humanoid-sim --features simulator --release

# Connect to Unitree robot (update IP)
connect ip="192.168.123.104":
    cargo run --bin unitree-bridge --features unitree -- {{ip}}

# Run example
example name="stand_up":
    cargo run --example {{name}} --features simulator

# Format code
fmt:
    cargo fmt

# Lint code
lint:
    cargo clippy --all-features -- -D warnings

# Generate documentation
docs:
    cargo doc --all-features --open

# Clean build artifacts
clean:
    cargo clean

# Run benchmarks
bench:
    cargo bench --features simulator

# Check all targets
check:
    cargo check --all-features
    cargo check --all-targets --all-features

# Install dependencies (Ubuntu/Debian)
install-deps:
    sudo apt-get update
    sudo apt-get install -y cmake g++ build-essential \
        libx11-dev libasound2-dev libudev-dev libwayland-dev libxkbcommon-dev

# Setup development environment
setup: install-deps
    rustup component add rustfmt clippy
    cargo install cargo-watch cargo-expand
    @echo "Setup complete! Run 'just sim' to start the simulator."

# Watch for changes and rebuild
watch:
    cargo watch -x "run --bin humanoid-sim --features simulator"
