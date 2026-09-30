# Smoke test for the installed package: consumes unitree_kinematics exactly as a
# downstream project would, via find_package() against the install prefix.
#
#   cmake -S tests/find_package -B build-test -DCMAKE_PREFIX_PATH=<install prefix>
#   cmake --build build-test
#   ./build-test/find_package_test
cmake_minimum_required(VERSION 3.16)

project(unitree_kinematics_find_package_test LANGUAGES CXX)

find_package(unitree_kinematics 0.1 REQUIRED)

add_executable(find_package_test main.cpp)
target_link_libraries(find_package_test PRIVATE unitree_kinematics::unitree_kinematics)
