# 🧮 MIQP_solver

A simple and naive implementation of a Mixed Integer Quadratic Program (MIQP) solver. This repository includes two separate implementations:

- A C++ version that uses QSQP and QSQP-Eigen
- A basic Python version provided for demonstration purposes only

## Requirements

### C++

- C++17
- [Eigen3](https://eigen.tuxfamily.org/)
- [QSQP](https://osqp.org/)
- [QSQP-Eigen](https://github.com/robotology/osqp-eigen)
- CMake >= 3.10


## Building and Running the C++ Solver

```bash
mkdir build && cd build
cmake ..
make
./miqp_solver
./tests/miqp_tests
```

## Disclaimer
This is a basic and naive implementation for demonstration and testing purposes only. It is not optimized for performance or production use.
