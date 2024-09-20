# Grains3DGPU

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Version](https://img.shields.io/badge/version-1.0.0-green.svg)

Grains3DGPU is a high-performance simulation framework for modeling the dynamics of granular materials using the Discrete Element Method (DEM) accelerated by Graphics Processing Units (GPUs). It supports complex particle shapes, advanced collision detection algorithms, and multi-GPU setups for large-scale simulations.

## Table of Contents
- [Features](#features)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
  - [Basic Example](#basic-example)
  - [Configuration](#configuration)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Features

- **GPU Acceleration**: Leverages CUDA for high-performance parallel computing, enabling large-scale simulations.
- **Advanced Collision Detection**: Implements Nesterov-accelerated GJK algorithm for accurate and efficient collision detection.
- **Support for Complex Particle Shapes**: Simulates non-spherical particles, including superquadrics and polyhedra.
- **Multi-GPU Support**: Distributes workload across multiple GPUs for even larger and more complex simulations.
- **User-Friendly API**: Intuitive interface for configuring simulations and retrieving results.

## Getting Started

### Prerequisites

- **CUDA Toolkit**: Version 11.0 or higher.
- **NVIDIA GPU**: Compute Capability 7.5 or higher is recommended.
- **C++ Compiler**: GCC or MSVC compatible with C++17.
- **Python**: For optional scripting and visualization support (version 3.6 or higher).

### Installation

1. **Clone the repository:**
    ```bash
    git clone https://github.com/AliRY95/Grains3DGPU
    cd Grains3DGPU
    ```

2. **Build the project:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

## Usage
