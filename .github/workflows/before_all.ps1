# OMPL Windows vcpkg setup script for cibuildwheel
Write-Host "Running before_all.ps1 for Windows"

# Check if vcpkg already exists
if (Test-Path "$PWD\vcpkg\vcpkg.exe") {
    Write-Host "vcpkg already exists"
    $env:VCPKG_ROOT = "$PWD\vcpkg"
} else {
    Write-Host "Downloading and bootstrapping vcpkg..."
    
    # Download vcpkg
    git clone https://github.com/Microsoft/vcpkg.git
    Set-Location vcpkg
    
    # Bootstrap vcpkg
    .\bootstrap-vcpkg.bat
    
    # Set VCPKG_ROOT environment variable
    Set-Location ..
    $env:VCPKG_ROOT = "$PWD\vcpkg"
}

# Set CMAKE_TOOLCHAIN_FILE environment variable
$env:CMAKE_TOOLCHAIN_FILE = "$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
Write-Host "VCPKG_ROOT=$env:VCPKG_ROOT"
Write-Host "CMAKE_TOOLCHAIN_FILE=$env:CMAKE_TOOLCHAIN_FILE"

# Export environment variables to parent process
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", $env:VCPKG_ROOT, "Process")
[Environment]::SetEnvironmentVariable("CMAKE_TOOLCHAIN_FILE", $env:CMAKE_TOOLCHAIN_FILE, "Process")

Write-Host "Installing dependencies with vcpkg..."
Set-Location $env:VCPKG_ROOT
# Use full path to call vcpkg executable
& .\vcpkg install --triplet=x64-windows

Set-Location ..
Write-Host "Finished vcpkg setup and dependency installation"