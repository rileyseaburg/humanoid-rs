# Versioning Policy

This document describes the versioning policy for humanoid-rs.

## Semantic Versioning

This project follows [Semantic Versioning 2.0.0](https://semver.org/):

- **MAJOR**: Incompatible API changes
- **MINOR**: New functionality, backwards compatible
- **PATCH**: Bug fixes, backwards compatible

Example: `0.2.0` -> `0.3.0` (minor) vs `0.2.0` -> `1.0.0` (major)

## API Stability Guarantees

### Current Status (0.x)

During 0.x development:
- APIs may change between minor versions
- Breaking changes will be documented in CHANGELOG.md
- Deprecation warnings will be provided when possible

### 1.0+ Stability

After 1.0.0 release:
- Public APIs will maintain backwards compatibility within major versions
- Breaking changes only in major version bumps
- All deprecations will follow the deprecation process

### What Constitutes Public API

Public API includes:
- Public types and traits in the crate root
- Public methods and functions
- Public constants
- Re-exported items
- Documented behavior

Not considered public API:
- Items marked with `#[doc(hidden)]`
- Items in private modules
- Undocumented behavior
- Internal implementation details

## Deprecation Process

### Timeline

1. **Deprecation Warning** (Minor version N)
   - Mark API with `#[deprecated]` attribute
   - Include migration guide in documentation
   - Update CHANGELOG.md

2. **Grace Period** (Minor versions N, N+1)
   - Deprecated API continues to work
   - Warnings displayed during compilation

3. **Removal** (Major version N+1 or N+2)
   - Remove deprecated API
   - Document in CHANGELOG.md

### Example

```rust
// In version 0.2.0
#[deprecated(
    since = "0.2.0",
    note = "Use `new_control_interface()` instead. \
            See migration guide: https://docs.rs/humanoid-rs/0.2.0/migration.html"
)]
pub fn old_control_interface() { ... }

// New API
pub fn new_control_interface() { ... }
```

## Version Support

### Active Support

- **Latest minor version**: Full support, bug fixes, security patches
- **Previous minor version**: Critical bug fixes and security patches only
- **Older versions**: No support

### Rust Version Policy

- MSRV (Minimum Supported Rust Version): 1.75.0
- MSRV bumps considered minor changes
- New features may require newer Rust versions

## Release Cycle

### Regular Releases

- **Patch releases**: As needed for bug fixes
- **Minor releases**: Approximately every 2-3 months
- **Major releases**: When breaking changes accumulate

### Release Process

1. Update version in `Cargo.toml`
2. Update `CHANGELOG.md`
3. Run release checks: `./scripts/release.sh`
4. Create git tag: `git tag vX.Y.Z`
5. Push tag: `git push origin vX.Y.Z`
6. GitHub Actions automatically publishes to crates.io

## Breaking Changes

### What Counts as Breaking

- Removing public items
- Changing function signatures
- Changing trait implementations
- Changing public struct field visibility
- Changing enum variants
- Changing documented behavior

### What Does NOT Count as Breaking

- Adding new items (types, functions, etc.)
- Adding new trait implementations
- Fixing bugs that change behavior to match documentation
- Adding new cargo features
- Performance improvements

### Documenting Breaking Changes

All breaking changes must be documented in:
1. CHANGELOG.md under "Breaking Changes" section
2. Release notes on GitHub
3. Migration guide when significant

## Pre-Release Versions

Pre-release versions may be published with suffixes:
- `1.0.0-alpha.1`
- `1.0.0-beta.2`
- `1.0.0-rc.1`

Pre-releases:
- Are not considered stable
- May contain bugs
- APIs may change before final release
- Not recommended for production use

## Questions?

For questions about versioning:
- Check existing [Issues](https://github.com/rileyseaburg/humanoid-rs/issues)
- Open a new discussion

---

Last updated: 2026-01-28
