# Code Coverage Notes

## Coverage Calculation Differences Between CI Environments

### Issue Background

The libgraph CI pipeline initially showed different coverage percentages between Ubuntu 22.04 and 24.04 environments due to different lcov configurations.

### Root Cause

**lcov Version Differences:**
- Ubuntu 22.04: lcov 1.14 (from package lcov 1.15-1 but reports 1.14 due to known packaging issue)
- Ubuntu 24.04: lcov 2.1 (from package lcov 2.0-4ubuntu2)

**Original CI Configuration Differences:**
- Ubuntu 24.04: Used `--rc geninfo_unexecuted_blocks=1` flag (supported in lcov 2.0+)
- Ubuntu 22.04: Used basic lcov command without this flag (not supported in lcov 1.x)

### Technical Impact

The `--rc geninfo_unexecuted_blocks=1` flag (available in lcov 2.0+) affects coverage calculation by:

1. **Template Instantiation Coverage**: Includes unexecuted template code paths in coverage calculations
2. **Inline Function Analysis**: More granular analysis of header-only library inline functions  
3. **Compiler Optimization Handling**: Different treatment of compiler-optimized or dead-code-eliminated blocks
4. **Exception Path Coverage**: Stricter accounting of unreachable exception handling blocks

### Solution

**Version-Aware Configuration (Applied to All Environments):**
```bash
# Check lcov version and use appropriate flags
LCOV_VERSION=$(lcov --version 2>&1 | grep -oP 'lcov version \K[0-9]+\.[0-9]+' || echo "0.0")

# lcov 2.0+ supports geninfo_unexecuted_blocks, older versions (1.x) do not
if [ "$(printf '%s\n' "2.0" "$LCOV_VERSION" | sort -V | head -n1)" = "2.0" ]; then
  # Use enhanced flags for lcov 2.0+
  lcov --directory ./build --capture --output-file ./build/coverage.info \
      --rc geninfo_unexecuted_blocks=1 --ignore-errors mismatch,negative,unused
else
  # Use compatible flags for lcov 1.x
  lcov --directory ./build --capture --output-file ./build/coverage.info \
      --ignore-errors mismatch,negative,unused
fi
```

### Benefits

- **Version Compatibility**: CI works correctly on both lcov 1.x and 2.x installations
- **Consistent Reporting**: Coverage calculations are appropriate for each lcov version
- **Enhanced Analysis**: lcov 2.0+ environments get more comprehensive template-aware coverage analysis
- **Robust Error Handling**: Enhanced error handling for various lcov edge cases across versions
- **Backwards Compatibility**: Maintains functionality on older Ubuntu/lcov installations

### Version-Specific Coverage Behavior

**lcov 2.0+ (Ubuntu 24.04):**
- More granular template instantiation coverage analysis
- Stricter accounting of inline functions in header-only libraries
- Enhanced compiler optimization and dead-code handling

**lcov 1.x (Ubuntu 22.04):**
- Traditional coverage analysis suitable for most projects
- Reliable baseline coverage metrics without advanced template handling

### Recommendations for Other Projects

For C++ projects using CI across multiple Ubuntu versions:
- Implement version detection to use appropriate lcov flags
- Consider that lcov 2.0+ provides stricter analysis that may lower coverage percentages
- Test coverage thresholds should account for version-specific behavior differences
- Document expected coverage variations between lcov versions

### Historical Context

This version-aware coverage solution was implemented in August 2025 to resolve CI discrepancies between Ubuntu 22.04 (lcov 1.14) and Ubuntu 24.04 (lcov 2.1) environments. The solution automatically detects lcov version and uses appropriate flags to ensure robust coverage reporting across different Ubuntu versions while maximizing analysis quality where possible.