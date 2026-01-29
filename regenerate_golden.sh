#!/usr/bin/env bash
#
# Regenerate golden files for deterministic tests
#
# Usage: ./regenerate_golden.sh [test_name]
#   If test_name is provided, only regenerate that specific golden file
#   Otherwise, regenerate all golden files

set -e

echo "Regenerating golden files..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Create golden directory if it doesn't exist
mkdir -p tests/golden

# Function to regenerate a specific test
test_name=$1

if [ -n "$test_name" ]; then
    echo -e "${YELLOW}Regenerating single golden file: $test_name${NC}"
    cargo test --test determinism $test_name --features simulator -- --nocapture 2>&1 | grep -i "created golden file" || true
else
    echo -e "${YELLOW}Regenerating all golden files...${NC}"
    
    # Run determinism tests which will create golden files if they don't exist
    cargo test --test determinism --features simulator -- --nocapture 2>&1 | tee /tmp/test_output.log || true
    
    # Check which golden files were created
    echo ""
    echo -e "${GREEN}Golden file generation complete${NC}"
    echo ""
    echo "Current golden files:"
    ls -lh tests/golden/*.json 2>/dev/null || echo "No golden files found"
fi

# Verify golden files exist
echo ""
echo "Verifying golden files..."

MISSING=0
for expected in "physics_golden_100steps" "standup_trajectory_golden"; do
    if [ ! -f "tests/golden/${expected}.json" ]; then
        echo -e "${RED}Missing: ${expected}.json${NC}"
        MISSING=1
    else
        echo -e "${GREEN}Found: ${expected}.json${NC}"
    fi
done

if [ $MISSING -eq 0 ]; then
    echo ""
    echo -e "${GREEN}All golden files present!${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}Some golden files are missing. Run tests to generate them.${NC}"
    exit 1
fi
