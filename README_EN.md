# Model Predictive Control (MPC) Implementation Collection

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

> **[English README](README_EN.md)** | **[日本語ドキュメント](README.md)**

MATLAB implementations of Model Predictive Control (MPC) for double integrator systems, including trajectory tracking, constrained MPC, and performance comparison with LQR.

## Overview

This repository provides educational MATLAB code for learning MPC theory and implementation. It features a double integrator system (position-velocity) with trajectory tracking control, constrained MPC, comparison with MATLAB MPC Toolbox, and performance benchmarking against LQR.

## Key Features

- **Manual QP Implementation** - Complete hand-coded MPC using Quadratic Programming
- **Constraint Handling** - MPC with input, state, and rate constraints
- **Toolbox Validation** - Verification against MATLAB MPC Toolbox
- **Performance Comparison** - Quantitative evaluation of MPC vs LQR
- **Dynamic Trajectory Tracking** - Time-varying control with sinusoidal reference trajectories
- **Comprehensive Documentation** - Complete theoretical background and implementation details

---

## Requirements

### Essential

- **MATLAB** R2020b or later
- **Optimization Toolbox** (for `quadprog` function)

### Optional

- **Control System Toolbox** (for system analysis and LQR comparison)
- **MPC Toolbox** (only for `MPC_Toolbox.m`)

---

## Installation

```bash
# Clone the repository
git clone https://github.com/youkoutaku/MPC-MATLAB-Examples.git
cd MPC-MATLAB-Examples
```

Start MATLAB and navigate to the project folder:

```matlab
cd('path/to/MPC-MATLAB-Examples')
```

Add all functions to the MATLAB path:
```matlab
addpath('lib')
addpath('test')
```

---

## Quick Start

### Simple Example: Trajectory Tracking MPC

```matlab
% Run from project folder
addpath('lib')  % Add helper functions to path
run('MPC_trajectory_main.m')
```

**Result:** Displays position and velocity plots tracking a sinusoidal reference trajectory.

### Constrained MPC (Recommended)

```matlab
% Execute constrained MPC
addpath('lib')  % First time only
run('MPC_con_main.m')

% Verify constraint satisfaction
run('test/Check_Constraints.m')
```

**Result:** Shows control results with input/velocity/position constraints and constraint activation rates.

### MPC vs LQR Comparison

```matlab
run('MPCvsLQR_QP_main.m')
```

**Result:** Side-by-side comparison of trajectory tracking performance.

---

## References

- [documents](docs/docs_en.pdf)
- Wang, L. (2009). *Model Predictive Control System Design and Implementation Using MATLAB*. Springer.
- Camacho, E. F., & Bordons, C. (2007). *Model Predictive Control*. Springer.
- Otsuka, T. (2011). *Introduction to Nonlinear Optimal Control*. Corona Publishing (Japanese).
- Wang, T. (2023). *The Beauty of Control (Volume 2)*. Tsinghua University Press (Chinese).

---

## Issues & Questions

- **Issues**: [GitHub Issues](https://github.com/youkoutaku/MPC-MATLAB-Examples/issues)
- **Email**: Feel free to send questions or suggestions

---

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.

---

## Author

**Youkoutaku**

- GitHub: [@Youkoutaku](https://github.com/Youkoutaku)
- Website: [https://youkoutaku.github.io/](https://youkoutaku.github.io/)

---

<div align="center">

**⭐ If this repository is helpful, please give it a star! ⭐**

Last updated: December 15, 2025

Edited by LLM AI

</div>
