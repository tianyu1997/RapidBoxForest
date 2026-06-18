# Testing

Run the integrated test suite from this module root:

```bash
bash tests/run_all.sh
```

Useful environment variables:

- `BUILD_DIR`: override build directory.
- `PYTHON_EXECUTABLE`: choose the Python interpreter used by CMake/pybind11.
- `RBF_WITH_PYTHON`: build Python bindings for enabled workspace modules.

The test script configures the workspace root and also runs an independence grep
check to catch accidental includes of forbidden private forest/planner/ffb/lect
headers.
