#!/usr/bin/env bash
# Create eight phase commits for the python bindings gap work.
set -euo pipefail
cd "$(dirname "$0")/.."

git checkout -B feat/python-bindings-gap 2>/dev/null || git checkout feat/python-bindings-gap

commit_phase() {
  local subject="$1"
  local body="$2"
  shift 2
  if git diff --quiet && git diff --cached --quiet && [ $# -eq 0 ]; then
    echo "Nothing to commit for: $subject"
    return
  fi
  git add "$@"
  if git diff --cached --quiet; then
    echo "No staged changes for: $subject"
    return
  fi
  git commit -m "$(cat <<EOF
$subject

$body
EOF
)"
}

# Phase 0
commit_phase \
  "feat(pybind): add binding gap tooling and gil exclusion docs" \
  "Add list_binding_gaps.py, planner binding skeleton, and document GIL-unsafe APIs excluded from Python." \
  scripts/list_binding_gaps.py \
  scripts/commit_binding_phases.sh \
  py-bindings/templates/PlannerSkeleton.cpp \
  doc/markdown/python.md

# Phase 1
commit_phase \
  "feat(pybind): bind base samplers objectives and util helpers" \
  "Expose base objectives, samplers, state spaces, validators, and util helpers needed by planner smoke tests." \
  py-bindings/util/ProlateHyperspheroid.cpp \
  py-bindings/util/GeometricEquations.cpp \
  py-bindings/util/Time.cpp \
  py-bindings/util/Exception.cpp \
  py-bindings/util/init.h \
  py-bindings/base/objectives/MinimizeArrivalTime.cpp \
  py-bindings/base/objectives/MechanicalWorkOptimizationObjective.cpp \
  py-bindings/base/objectives/VFMechanicalWorkOptimizationObjective.cpp \
  py-bindings/base/objectives/MinimaxObjective.cpp \
  py-bindings/base/objectives/MaximizeMinClearanceObjective.cpp \
  py-bindings/base/objectives/ControlDurationObjective.cpp \
  py-bindings/base/objectives/VFUpstreamCriterionOptimizationObjective.cpp \
  py-bindings/base/terminationconditions/ \
  py-bindings/base/samplers/GaussianValidStateSampler.cpp \
  py-bindings/base/samplers/BridgeTestValidStateSampler.cpp \
  py-bindings/base/samplers/MinimumClearanceValidStateSampler.cpp \
  py-bindings/base/samplers/MaximizeClearanceValidStateSampler.cpp \
  py-bindings/base/samplers/ConditionalStateSampler.cpp \
  py-bindings/base/samplers/InformedStateSampler.cpp \
  py-bindings/base/samplers/RejectionInfSampler.cpp \
  py-bindings/base/samplers/OrderedInfSampler.cpp \
  py-bindings/base/samplers/PathLengthDirectInfSampler.cpp \
  py-bindings/base/samplers/DeterministicStateSampler.cpp \
  py-bindings/base/samplers/deterministic/ \
  py-bindings/base/spaces/HybridStateSpace.cpp \
  py-bindings/base/spaces/HybridTimeStateSpace.cpp \
  py-bindings/base/spaces/special/ \
  py-bindings/base/DubinsMotionValidator.cpp \
  py-bindings/base/Dubins3DMotionValidator.cpp \
  py-bindings/base/DiscreteMotionValidator.cpp \
  py-bindings/base/GenericParam.cpp \
  py-bindings/base/PrecomputedStateSampler.cpp \
  py-bindings/base/StateStorage.cpp \
  py-bindings/base/PlannerDataGraph.cpp \
  py-bindings/base/goals/GoalLazySamples.cpp \
  py-bindings/base/spaces/DubinsStateSpace.cpp \
  py-bindings/base/spaces/ReedsSheppStateSpace.cpp \
  py-bindings/base/spaces/TrochoidStateSpace.cpp \
  py-bindings/base/spaces/OwenStateSpace.cpp \
  py-bindings/base/spaces/VanaStateSpace.cpp \
  py-bindings/base/spaces/VanaOwenStateSpace.cpp \
  py-bindings/base/init.h

# Phase 2
commit_phase \
  "feat(pybind): bind geometric planners and path utilities" \
  "Add classic geometric planners, path utilities, and smoke-test coverage for EST, LazyRRT, and related RRT variants." \
  py-bindings/geometric/PathHybridization.cpp \
  py-bindings/geometric/HillClimbing.cpp \
  py-bindings/geometric/GeneticSearch.cpp \
  py-bindings/geometric/planners/est/ \
  py-bindings/geometric/planners/sst/SST.cpp \
  py-bindings/geometric/planners/sbl/ \
  py-bindings/geometric/planners/pdst/PDST.cpp \
  py-bindings/geometric/planners/stride/ \
  py-bindings/geometric/planners/rrt/LazyRRT.cpp \
  py-bindings/geometric/planners/rrt/LBTRRT.cpp \
  py-bindings/geometric/planners/rrt/LazyLBTRRT.cpp \
  py-bindings/geometric/planners/rrt/TRRT.cpp \
  py-bindings/geometric/planners/rrt/TRRTstar.cpp \
  py-bindings/geometric/planners/rrt/BiTRRT.cpp \
  py-bindings/geometric/planners/rrt/TSRRT.cpp \
  py-bindings/geometric/planners/rrt/ATRRT.cpp \
  py-bindings/geometric/planners/rrt/VFRRT.cpp \
  py-bindings/geometric/planners/rrt/STRRTstar.cpp \
  py-bindings/geometric/planners/rrt/RRTXstatic.cpp \
  py-bindings/geometric/planners/rrt/RRTsharp.cpp \
  py-bindings/geometric/planners/rrt/AOXRRTConnect.cpp \
  py-bindings/geometric/planners/rlrt/ \
  py-bindings/geometric/planners/prm/LazyPRM.cpp \
  py-bindings/geometric/planners/prm/LazyPRMstar.cpp \
  py-bindings/geometric/planners/prm/SPARS.cpp \
  py-bindings/geometric/planners/prm/SPARStwo.cpp \
  py-bindings/geometric/init.h

# Phase 3
commit_phase \
  "feat(pybind): bind informed-tree geometric planners" \
  "Register informed-tree planners in BITstar inheritance order for nanobind trampolines." \
  py-bindings/geometric/planners/informedtrees/ABITstar.cpp \
  py-bindings/geometric/planners/informedtrees/AITstar.cpp \
  py-bindings/geometric/planners/informedtrees/EITstar.cpp \
  py-bindings/geometric/planners/informedtrees/EIRMstar.cpp \
  py-bindings/geometric/planners/lazyinformedtrees/

# Phase 4
commit_phase \
  "feat(pybind): bind control ode ltl and hybrid planners" \
  "Bind ODE solvers, hybrid planners, and the LTL stack while skipping ODEAdaptiveSolver on Boost 1.83." \
  py-bindings/control/ODESolver.cpp \
  py-bindings/control/spaces/DiscreteControlSpace.cpp \
  py-bindings/control/SteeredControlSampler.cpp \
  py-bindings/control/PlannerDataStorage.cpp \
  py-bindings/control/planners/rrt/HyRRT.cpp \
  py-bindings/control/planners/sst/HySST.cpp \
  py-bindings/control/planners/ltl/ \
  py-bindings/control/init.h

# Phase 5
commit_phase \
  "feat(pybind): bind tools experience and xxl helpers" \
  "Add tools, experience DB, and XXL decomposition bindings used by planner demos and tests." \
  py-bindings/tools/config/ \
  py-bindings/tools/lightning/ \
  py-bindings/tools/thunder/ \
  py-bindings/tools/benchmark/MachineSpecs.cpp \
  py-bindings/tools/init.h \
  py-bindings/geometric/planners/experience/ \
  py-bindings/geometric/planners/xxl/

# Phase 6
commit_phase \
  "feat(pybind): add multilevel submodule bindings" \
  "Scaffold multilevel projections, parameters, bundle-space infra, and QRRT/QMP planners." \
  py-bindings/multilevel/ \
  py-bindings/ompl/multilevel.py \
  py-bindings/CMakeLists.txt

# Phase 7 — registration, tests, docs
commit_phase \
  "feat(pybind): wire registrations tests and binding docs" \
  "Register all new bindings in python.cpp and add geo/multilevel pytest coverage plus planner docs." \
  py-bindings/python.cpp \
  py-bindings/ompl/__init__.py \
  tests/pytests/test_geo_planners.py \
  tests/pytests/test_multilevel_planners.py \
  doc/markdown/pybindingsPlanner.md \
  docs/plans/python-bindings-execution.plan.md \
  docs/plans/ompl-python-bindings-gap.plan.md

echo "Done. Log:"
git log --oneline -8
