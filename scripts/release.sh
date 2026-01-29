#!/bin/bash
set -e

echo "========================================"
echo "Running release checks for humanoid-rs"
echo "========================================"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're on a clean git state
if [ -n "$(git status --porcelain)" ]; then
    echo -e "${RED}Error: Working directory is not clean${NC}"
    echo "Please commit or stash changes before running release checks"
    exit 1
fi

echo -e "${YELLOW}1. Running tests...${NC}"
cargo test --all-features --quiet
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Tests passed${NC}"
else
    echo -e "${RED}✗ Tests failed${NC}"
    exit 1
fi

echo -e "${YELLOW}2. Running clippy...${NC}"
cargo clippy --all-features -- -D warnings --quiet
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Clippy passed${NC}"
else
    echo -e "${RED}✗ Clippy failed${NC}"
    exit 1
fi

echo -e "${YELLOW}3. Checking formatting...${NC}"
cargo fmt --check
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Formatting check passed${NC}"
else
    echo -e "${RED}✗ Formatting check failed${NC}"
    echo "Run 'cargo fmt' to fix formatting"
    exit 1
fi

echo -e "${YELLOW}4. Generating documentation...${NC}"
cargo doc --all-features --no-deps --quiet 2>&1 | grep -E "^error" && exit 1 || true
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Documentation generated${NC}"
else
    echo -e "${RED}✗ Documentation generation failed${NC}"
    exit 1
fi

echo -e "${YELLOW}5. Running publish dry-run...${NC}"
cargo publish --dry-run --quiet 2>&1
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Publish dry-run passed${NC}"
else
    echo -e "${RED}✗ Publish dry-run failed${NC}"
    exit 1
fi

# Check version in Cargo.toml matches git tag (if tag exists)
if [ -n "$1" ]; then
    CARGO_VERSION=$(grep "^version" Cargo.toml | head -1 | sed 's/.*"\(.*\)".*/\1/')
    if [ "$1" = "v$CARGO_VERSION" ]; then
        echo -e "${GREEN}✓ Version matches: $CARGO_VERSION${NC}"
    else
        echo -e "${RED}✗ Version mismatch!${NC}"
        echo "  Tag: $1"
        echo "  Cargo.toml: $CARGO_VERSION"
        exit 1
    fi
fi

echo ""
echo "========================================"
echo -e "${GREEN}All checks passed!${NC}"
echo "========================================"
echo ""

if [ -n "$1" ]; then
    echo "Ready to publish version $1"
    echo ""
    echo "To publish, run:"
    echo "  git tag $1"
    echo "  git push origin $1"
    echo ""
    echo "Or manually run: cargo publish"
else
    echo "Ready for release."
    echo ""
    echo "When ready:"
    echo "1. Update version in Cargo.toml"
    echo "2. Update CHANGELOG.md"
    echo "3. Commit changes"
    echo "4. Run: ./scripts/release.sh v<version>"
    echo "5. git tag v<version> && git push origin v<version>"
fi

exit 0
