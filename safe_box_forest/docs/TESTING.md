# Testing

Run the standalone test suite from this package root:

```bash
bash tests/run_all.sh
```

Useful environment variables:

- `BUILD_DIR`: override build directory.
- `PYTHON_EXECUTABLE`: choose the Python interpreter used by CMake/pybind11.
- `SBF_LECTDATABASE_SOURCE_DIR`: path to the consolidated LECTDatabase package.
- `SBF_LINK_INTERVAL_ENVELOPE_SOURCE_DIR`: path to standalone link interval envelope.

The test script also runs an independence grep check to catch accidental
includes of v6-only forest/planner/ffb/lect headers.