# Development environment

Supported baseline:

- Ubuntu 22.04 or 24.04
- GCC 11+ or Clang 14+
- CMake 3.22+
- Ninja 1.10+ recommended

## Build and test

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DDRONE_LAB_BUILD_TESTS=ON \
  -DDRONE_LAB_WARNINGS_AS_ERRORS=ON
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix build/install
```

After installation, downstream CMake projects can use `find_package(DroneLab CONFIG REQUIRED)` and link `DroneLab::Core`.
