#--------------------------------------------------------------------------------------------------
# This is a small test project to make sure that EbsdLib can be successfully used
# from another completely independent C++ CMake based project. If you want to test
# this out do the following:
# Build and INSTALL EbsdLib itself first.
# I would suggest the following:
# ```
# cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -DHDF5_DIR=/Users/Shared/DREAM3D_SDK/hdf5-1.10.7-Release/share/cmake/hdf5 -DTBB_DIR=/Users/Shared/DREAM3D_SDK/oneTBB-2021.4.0-Release/lib/cmake/TBB -DCMAKE_INSTALL_PREFIX=/tmp/ebsdlib ../../EbsdLib
# cmake --build --target install
# ```
#
# Now that you have EbsdLib installed you can try and build this small project:
# ```
# cd EbsdLib/Test/ProjectTest
# mkdir Build && cd Build
# cmake -DEbsdlib_DIR=/tmp/ebsdlib/share/EbsdLib -DH5Support_DIR=/tmp/ebsdlib/share/H5Support -DHDF5_DIR=/Users/Shared/DREAM3D_SDK/hdf5-1.10.7-Release/share/cmake/hdf5 -DTBB_DIR=/Users/Shared/DREAM3D_SDK/oneTBB-2021.4.0-Release/lib/cmake/TBB  ../
# cmake --build .
# ./ebsdlib_external_project_test
# ```
# That should project a tiff image located at 'Triclinic IPF Legend/Triclinic -1 (Ci)/Triclinic -1 (Ci)_FULL.tiff'

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0054 NEW)
cmake_policy(SET CMP0079 NEW)
cmake_policy(SET CMP0077 NEW)

cmake_minimum_required(VERSION 3.20.0)

#--------------------------------------------------------------------------------------------------
# Request C++17 standard, using new CMake variables.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#--------------------------------------------------------------------------------------------------
# Always write out the compile_commands.json file to help out things like QtCreator and VS Code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#--------------------------------------------------------------------------------------------------
# Create the project
project(EbsdLibExternalTest VERSION 1.0)

find_package(Ebsdlib REQUIRED)

add_executable(ebsdlib_external_project_test  ebsdlib_external_project_test.cpp)
target_link_libraries(ebsdlib_external_project_test EbsdLib::EbsdLib)
