#!/usr/bin/env python3
"""Diff public C++ headers under src/ompl/ against py-bindings/**/*.cpp."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


SKIP_HEADER_PARTS = (
    "/datastructures/",
    "Impl.h",
    "Head.h",
    "Vertex.h",
    "Edge.h",
    "FindSectionTypes.h",
    "PathSection.h",
    "ClassForward.h",
    "DisableCompilerWarning.h",
    "Hash.h",
    "String.h",
    "PlannerIncludes.h",
    "ConnectionStrategy.h",
    "ImplicitGraph.h",
    "Queuetypes.h",
    "CostHelper.h",
    "HelperFunctions.h",
    "IdGenerator.h",
    "SearchQueue.h",
    "Direction.h",
    "ForwardQueue.h",
    "ReverseQueue.h",
    "RandomGeometricGraph.h",
)

EXCLUDED_CLASSES = {
    "ParallelPlan",
    "OptimizePlan",
    "CForest",
    "CForestStateSpaceWrapper",
    "CForestStateSampler",
    "pRRT",
    "pSBL",
    "AnytimePathShortening",
    "Lightning",
    "Thunder",
    "ExperienceSetup",
    "Profiler",
    "PlannerMonitor",
    # Boost 1.83 template issue; see py-bindings/control/ODESolver.cpp
    "ODEAdaptiveSolver",
}

# Public headers intentionally left unbound (templates, typedef-only, or C++-only helpers).
INTENTIONAL_UNBOUND = {
    "ScopedState",
    "SolutionNonExistenceProof",
    "StateSamplerArray",
    "StateSpaceTypes",
    "TypedSpaceInformation",
    "TypedStateValidityChecker",
}

MODULES = ("base", "geometric", "control", "util", "tools", "multilevel")

INCLUDE_RE = re.compile(r'#include\s+"ompl/[^"]+/([^"/]+)\.h"')


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def header_stem(path: Path) -> str:
    return path.stem


def binding_stems(bindings_dir: Path) -> set[str]:
    stems: set[str] = set()
    for cpp in bindings_dir.rglob("*.cpp"):
        if cpp.name == "python.cpp":
            continue
        stems.add(cpp.stem)
        try:
            text = cpp.read_text(encoding="utf-8")
        except OSError:
            continue
        for match in INCLUDE_RE.finditer(text):
            stems.add(match.group(1))
    return stems


def should_skip_header(path: Path) -> bool:
    text = str(path)
    return any(part in text for part in SKIP_HEADER_PARTS)


def collect_headers(src_ompl: Path, module: str) -> list[Path]:
    module_dir = src_ompl / module
    if not module_dir.is_dir():
        return []
    headers = []
    for path in sorted(module_dir.rglob("*.h")):
        if should_skip_header(path):
            continue
        if path.stem in EXCLUDED_CLASSES:
            continue
        if path.stem in INTENTIONAL_UNBOUND:
            continue
        headers.append(path)
    return headers


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--module",
        choices=MODULES,
        action="append",
        help="Limit report to one or more modules (default: all)",
    )
    parser.add_argument(
        "--fail-on-gap",
        action="store_true",
        help="Exit with code 1 if any gaps remain",
    )
    args = parser.parse_args()

    root = repo_root()
    src_ompl = root / "src" / "ompl"
    bindings_dir = root / "py-bindings"
    bound = binding_stems(bindings_dir)

    modules = args.module or list(MODULES)
    total_missing = 0

    for module in modules:
        headers = collect_headers(src_ompl, module)
        missing = [h for h in headers if header_stem(h) not in bound]
        print(f"\n== {module} ({len(missing)} missing / {len(headers)} headers) ==")
        for path in missing:
            rel = path.relative_to(src_ompl)
            print(f"  {rel}")
        total_missing += len(missing)

    print(f"\nTotal missing: {total_missing}")
    if args.fail_on_gap and total_missing:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
