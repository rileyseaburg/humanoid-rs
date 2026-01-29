# Contributing to Humanoid-RS

Thank you for your interest in contributing to humanoid-rs! This document provides guidelines and instructions for contributing.

## Table of Contents

- [Development Setup](#development-setup)
- [Pull Request Process](#pull-request-process)
- [Code Style Guidelines](#code-style-guidelines)
- [Testing Requirements](#testing-requirements)
- [Commit Message Guidelines](#commit-message-guidelines)

## Development Setup

### Prerequisites

- Rust 1.75.0 or later
- Git

### Setup Steps

1. Fork the repository on GitHub
2. Clone your fork:
   ```bash
   git clone https://github.com/YOUR_USERNAME/humanoid-rs.git
   cd humanoid-rs
   ```

3. Install development dependencies:
   ```bash
   rustup component add clippy rustfmt
   ```

4. Build the project:
   ```bash
   cargo build --all-features
   ```

5. Run tests:
   ```bash
   cargo test --all-features
   ```

## Pull Request Process

1. **Create a Branch**
   ```bash
   git checkout -b feature/your-feature-name
   # or
   git checkout -b fix/your-bug-fix
   ```

2. **Make Your Changes**
   - Write clean, documented code
   - Add tests for new functionality
   - Update documentation as needed

3. **Run Quality Checks**
   ```bash
   cargo test --all-features
   cargo clippy --all-features -- -D warnings
   cargo fmt --check
   cargo doc --all-features --no-deps
   ```

4. **Commit Your Changes**
   - Use clear, descriptive commit messages
   - Reference issues when applicable

5. **Push and Create PR**
   ```bash
   git push origin feature/your-feature-name
   ```
   Then create a pull request on GitHub.

6. **PR Requirements**
   - Fill out the PR template completely
   - Ensure all CI checks pass
   - Request review from maintainers
   - Address review feedback promptly

## Code Style Guidelines

### Rust Style

We follow the [Rust Style Guide](https://doc.rust-lang.org/style-guide/index.html) and enforce it with `rustfmt`.

- Run `cargo fmt` before committing
- Run `cargo clippy` and fix all warnings
- Maximum line length: 100 characters
- Use 4 spaces for indentation

### Documentation

- All public items must have doc comments
- Use `///` for item documentation
- Use `//!` for module documentation
- Include examples in doc comments when helpful
- Use complete sentences with proper punctuation

Example:
```rust
/// Calculates the forward kinematics for a robot joint.
///
/// # Arguments
///
/// * `joint_angles` - Joint angles in radians
///
/// # Returns
///
/// Returns the end-effector pose as a homogeneous transformation matrix.
///
/// # Example
///
/// ```
/// use humanoid_rs::kinematics::forward_kinematics;
///
/// let angles = [0.0, 0.5, -0.3];
/// let pose = forward_kinematics(&angles);
/// ```
pub fn forward_kinematics(joint_angles: &[f64]) -> Pose {
    // Implementation
}
```

### Naming Conventions

- `snake_case` for functions, variables, and modules
- `PascalCase` for types, traits, and enum variants
- `SCREAMING_SNAKE_CASE` for constants and statics
- Avoid single-letter names except for loop indices

## Testing Requirements

### Test Coverage

- All new code must have tests
- Aim for >80% code coverage
- Use `cargo tarpaulin` to check coverage

### Test Types

1. **Unit Tests**: Test individual functions in isolation
   ```rust
   #[cfg(test)]
   mod tests {
       use super::*;

       #[test]
       fn test_joint_calculation() {
           let joint = Joint::new(1.0, -1.0);
           assert_eq!(joint.clamp(2.0), 1.0);
       }
   }
   ```

2. **Integration Tests**: Test component interactions
   - Place in `tests/` directory
   - Name files descriptively: `test_simulator_physics.rs`

3. **Documentation Tests**: Test code examples in docs
   - All examples in `///` comments should be testable

### Running Tests

```bash
# Run all tests
cargo test --all-features

# Run with output
cargo test --all-features -- --nocapture

# Run specific test
cargo test test_joint_calculation -- --nocapture

# Check test coverage
cargo tarpaulin --all-features
```

## Commit Message Guidelines

We follow the [Conventional Commits](https://www.conventionalcommits.org/) specification.

### Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Types

- **feat**: New feature
- **fix**: Bug fix
- **docs**: Documentation changes
- **style**: Code style changes (formatting, semicolons, etc.)
- **refactor**: Code refactoring
- **perf**: Performance improvements
- **test**: Adding or updating tests
- **chore**: Build process or auxiliary tool changes

### Examples

```
feat(simulator): add physics backend abstraction

Add support for multiple physics engines through a unified backend
interface. Currently supports Rapier and PhysX.

Closes #123
```

```
fix(control): correct joint limit checking

Fixed off-by-one error in joint limit validation that caused
incorrect clamping at boundary values.

Fixes #456
```

## Questions?

If you have questions or need help:
- Open a [Discussion](https://github.com/rileyseaburg/humanoid-rs/discussions)
- Join our [Discord/Slack]
- Email: maintainers@example.com

Thank you for contributing!
