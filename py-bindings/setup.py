#!/usr/bin/env python

import os
import re
import subprocess
import sys
import site
import shutil
import platform  # <-- Nuevo para detectar el sistema y la arquitectura
from pathlib import Path
from sysconfig import get_paths

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# Convert distutils Windows platform specifiers to CMake -A arguments
PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}


# ======================================================
# Funciones para identificar el sistema y la arquitectura
# ======================================================




def get_system() -> str:
    """
    Returns the system name as used in wheel filenames.
    """
    if platform.system() == "Windows":
        return "win"
    elif platform.system() == "Darwin":
         mac_version = ".".join(platform.mac_ver()[0].split(".")[:1])
         return f"macos_{mac_version}"
    elif platform.system() == "Linux":
         return "linux"
    else:
         raise ValueError("Unsupported system: {}".format(platform.system()))



def get_arch() -> str:
    """
    Returns the system name as used in wheel filenames.
    """
    if platform.machine() == "x86_64":
        if platform.system() == "Darwin":
            return "amd64"
        else:
            return "x86_64"
    elif platform.machine() == "arm64" or platform.machine() == "aarch64":
        if platform.system() == "Darwin":
            return "arm64"
        else:
            return "aarch64"
    else:
        raise ValueError("Unsupported arch: {}".format(platform.machine()))


def get_platform() -> str:
    """
    Devuelve el nombre de la plataforma (sistema y arquitectura)
    tal y como se usa en el nombre de las wheels.
    """
    return f"{get_system()}_{get_arch()}"


# ======================================================
# Fin funciones para sistema y arquitectura
# ======================================================


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        # Debido a un bug en .resolve() (corregido en Python 3.10+)
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        # Permite sobreescribir el generador con la variable de entorno CMAKE_GENERATOR
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        cmake_args = [
            f"-DCMAKE_CXX_COMPILER={shutil.which('clang++')}",  # Forzamos Clang para castxml
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXEC={sys.executable}",
            f"-DPYTHON_INCLUDE_DIRS={get_paths()['include']}",
            f"-DPYTHON_LIBRARIES={get_paths()['stdlib']}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            "-DOMPL_BUILD_PYBINDINGS=ON",
            "-DOMPL_REGISTRATION=OFF",
            "-DOMPL_BUILD_DEMOS=OFF",
            "-DOMPL_BUILD_PYTESTS=OFF",
            "-DOMPL_BUILD_TESTS=OFF",
        ]
        build_args = []

        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        if self.compiler.compiler_type != "msvc":
            # Usamos Ninja si es posible (no hace falta en MSVC)
            if not cmake_generator or cmake_generator == "Ninja":
                try:
                    import ninja
                    ninja_executable_path = Path(ninja.BIN_DIR) / "ninja"
                    cmake_args += [
                        "-GNinja",
                        f"-DCMAKE_MAKE_PROGRAM:FILEPATH={ninja_executable_path}",
                    ]
                except ImportError:
                    pass
        else:
            # En Windows usamos el generador de MSVC
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})
            if not single_config and not contains_arch:
                cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]
            if not single_config:
                cmake_args += [f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"]
                build_args += ["--config", cfg]

        if sys.platform.startswith("darwin"):
            # En macOS configuramos el deployment target
            cmake_args += ["-DCMAKE_OSX_DEPLOYMENT_TARGET=13.0"]
            # Soporte para cross-compiling: si ARCHFLAGS está definido, se utiliza;
            # de lo contrario se usa la función get_arch() para detectar la arquitectura.
            archs = re.findall(r"-arch (\S+)", os.environ.get("ARCHFLAGS", ""))
            if archs:
                cmake_args += ["-DCMAKE_OSX_ARCHITECTURES={}".format(";".join(archs))]
            else:
                cmake_args += ["-DCMAKE_OSX_ARCHITECTURES={}".format(get_arch())]

        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += [f"-j{self.parallel}"]

        build_temp = Path(self.build_temp) / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True)

        subprocess.run(
            ["cmake", ext.sourcedir, *cmake_args],
            cwd=build_temp,
            check=True
        )
        subprocess.run(["ninja", "update_bindings"], cwd=build_temp, check=True)
        subprocess.run(
            ["cmake", "--build", ".", *build_args],
            cwd=build_temp,
            check=True
        )

        # Copia las librerías compartidas (_base.so, _control.so, etc.)
        # a los directorios correspondientes dentro de ompl.
        for f in ["base", "control", "geometric", "tools", "util"]:
            subprocess.run(
                [f"cp {extdir}/_{f}.so {extdir}/ompl/{f}/"],
                cwd=build_temp,
                check=True,
                shell=True,
            )


setup(
    name="ompl",
    version="1.7.0",
    description="The Open Motion Planning Library",
    author="Ioan A. Șucan, Mark Moll, Zachary Kingston, Lydia E. Kavraki",
    author_email="zak@rice.edu",
    url="https://ompl.kavrakilab.org",
    ext_modules=[CMakeExtension("ompl", sourcedir="..")],
    cmdclass={"build_ext": CMakeBuild},
    packages=[
        "ompl",
        "ompl.base",
        "ompl.control",
        "ompl.geometric",
        "ompl.tools",
        "ompl.util",
    ],
    package_dir={"ompl": "./ompl"},
)