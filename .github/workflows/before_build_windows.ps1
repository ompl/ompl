$ErrorActionPreference = "Stop"

Write-Host "Installing dependencies with vcpkg..."

# Ensure we're targeting 64-bit
$env:VCPKG_DEFAULT_TRIPLET = "x64-windows"

# Install required libraries
vcpkg install `
    boost-serialization `
    boost-program-options `
    yaml-cpp `
    eigen3

# Ensure pip build env can see Ninja/CMake
python -m pip install --upgrade pip
python -m pip install cmake ninja

Write-Host "Dependencies installed successfully."
