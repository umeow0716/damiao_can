# Development

## How to build C++ Library

### Prerequisites

- CMake 3.22+
- C++17 compiler

### Build

```bash
cd damiao_can
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
```

## Build without CLI tools

If you only need the C++ library (or are building the Python bindings), disable the CLI dependency:

```bash
cmake -S . -B build -DDAMIAO_CAN_BUILD_CLI=OFF
cmake --build build
```
