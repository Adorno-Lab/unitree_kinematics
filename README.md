![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/Adorno-Lab/unitree_kinematics?color=orange)

# unitree_kinematics

[DQ Robotics](https://dqrobotics.github.io) kinematic models of Unitree robots, plus the matching CoppeliaSim (ZeroMQ remote API) robot class.


# Install

> [!NOTE]
> Non-sudo privileges? Create a custom prefix folder (e.g. `~/opt`) to hold `lib/` and `include/` without needing root. See [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html) for background.

## Prerequisites

- CMake ≥ 3.16 and a C++17 compiler.
- Eigen3: `sudo apt install libeigen3-dev` (macOS: `brew install eigen`)
- [DQ Robotics](https://dqrobotics.github.io) for C++. On Ubuntu:
  ```shell
  sudo add-apt-repository ppa:dqrobotics-dev/development -y
  sudo apt-get update
  sudo apt-get install libdqrobotics
  ```
- [DQ Robotics CoppeliaSim ZMQ interface](https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq). Build and install it by following its own instructions.

If you install any of these without sudo, put them in the same custom prefix (`~/opt`) and follow the non-sudo instructions below for `unitree_kinematics` as well. CMake then finds them all through `CMAKE_PREFIX_PATH`.

## Sudo users

```shell
git clone https://github.com/Adorno-Lab/unitree_kinematics.git
cd unitree_kinematics

# 1. Configure: choose Release, and (optionally) where to install it.
#    Omit -DCMAKE_INSTALL_PREFIX to use the system default (/usr/local on Linux).
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 2. Build the shared library.
cmake --build build -j$(nproc)

# 3. Install headers, library, and the exported CMake package.
sudo cmake --install build
```

> [!TIP]
> On Linux, refresh the linker cache after installing to a system prefix, so
> programs find `libunitree_kinematics.so` at runtime:
> ```shell
> sudo ldconfig
> ```

## Non-sudo users

```shell
git clone https://github.com/Adorno-Lab/unitree_kinematics.git
cd unitree_kinematics

# 1. Configure: choose Release, and install to your own prefix instead of a system path.
#    CMAKE_PREFIX_PATH lets CMake find dependencies you also installed there.
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/opt \
    -DCMAKE_PREFIX_PATH=$HOME/opt

# 2. Build the shared library.
cmake --build build -j$(nproc)

# 3. Install headers, library, and the exported CMake package. No sudo needed.
cmake --install build
```

> [!TIP]
> If you skipped exporting `CMAKE_PREFIX_PATH` (along with `LD_LIBRARY_PATH`,
> `LIBRARY_PATH`, and `CPATH`) in `~/.bashrc` (see [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html)),
> any project that later does `find_package(unitree_kinematics)` needs to be told
> where to look, since `$HOME/opt` isn't a default search path:
>
> ```shell
> cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/opt
> ```

## Uninstall

CMake records every installed file in `build/install_manifest.txt`:

```shell
xargs rm -v < build/install_manifest.txt   # prefix with sudo for system installs
```

# Usage

```cmake
find_package(unitree_kinematics REQUIRED)
target_link_libraries(${YOUR_TARGET} PRIVATE unitree_kinematics::unitree_kinematics)
```

The target carries its include directories and its DQ Robotics, CoppeliaSim ZMQ
interface, and Eigen3 dependencies, so you don't need to link those separately.

```cpp
#include <dqrobotics/robots/UnitreeZ1Robot.h>
#include <dqrobotics/robots/UnitreeB1Z1MobileRobot.h>
#include <dqrobotics/robots/CFFSerialRobot.h>
#include <dqrobotics/interfaces/coppeliasim/robots/UnitreeB1Z1CoppeliaSimZMQRobot.h>

using namespace DQ_robotics;

auto z1 = UnitreeZ1Robot::kinematics();          // DQ_SerialManipulatorDH, 6 DoF
UnitreeB1Z1MobileRobot b1z1;                     // 9 DoF (3 base + 6 arm)
```
