# Testing

Run the test suite from this module root:

```bash
bash tests/run_all.sh
```

Useful environment variables:

- `BUILD_DIR`: override build directory.
- `PYTHON_EXECUTABLE`: choose the Python interpreter used by CMake/pybind11.
- `SBF_LECTDATABASE_SOURCE_DIR`: path to the sibling `lect_database` module.

The test script also runs an independence grep check to catch accidental
includes of v6-only forest/planner/ffb/lect headers.