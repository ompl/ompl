#!/usr/bin/env python3
"""One-off generator for Phase 2 geometric bindings."""
import os
from textwrap import dedent

BASE = os.path.join(os.path.dirname(__file__))


def rel_init(depth):
    return "/".join([".."] * depth) + "/init.h"


def solve_lambda(class_name):
    return dedent(
        f"""
        .def("solve",
             [](og::{class_name} &self, nb::object what)
             {{
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {{
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 }}
                 else if (nb::isinstance<double>(what))
                 {{
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }}
                 else
                 {{
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }}
             }})"""
    )


def planner_data(class_name):
    return dedent(
        f"""
        .def(
            "getPlannerData", [](const og::{class_name} &self, ob::PlannerData &data) {{ self.getPlannerData(data); }},
            nb::arg("data"))"""
    )


def construct_roadmap_lambda(class_name, stop_arg=False):
    if stop_arg:
        return dedent(
            f"""
        .def("constructRoadmap",
             [](og::{class_name} &self, nb::object what, bool stopOnMaxFail)
             {{
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {{
                     return self.constructRoadmap(nb::cast<ob::PlannerTerminationCondition>(what), stopOnMaxFail);
                 }}
                 else if (nb::isinstance<double>(what))
                 {{
                     return self.constructRoadmap(ob::timedPlannerTerminationCondition(nb::cast<double>(what)), stopOnMaxFail);
                 }}
                 else
                 {{
                     throw nb::type_error(
                         "Invalid argument type for constructRoadmap. Expected PlannerTerminationCondition or double.");
                 }}
             }},
             nb::arg("ptc"), nb::arg("stopOnMaxFail") = false)"""
        )
    return dedent(
        f"""
        .def("constructRoadmap",
             [](og::{class_name} &self, nb::object what)
             {{
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {{
                     return self.constructRoadmap(nb::cast<ob::PlannerTerminationCondition>(what));
                 }}
                 else if (nb::isinstance<double>(what))
                 {{
                     return self.constructRoadmap(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }}
                 else
                 {{
                     throw nb::type_error(
                         "Invalid argument type for constructRoadmap. Expected PlannerTerminationCondition or double.");
                 }}
             }},
             nb::arg("ptc"))"""
    )


def fmt_method(cls, m):
    if len(m) == 1:
        name = m[0]
        return f'.def("{name}", &og::{cls}::{name})'
    if len(m) == 2:
        name, arg = m
        return f'.def("{name}", &og::{cls}::{name}, nb::arg("{arg}"))'
    name, arg, cast = m
    return f'.def("{name}", {cast}, nb::arg("{arg}"))'


def write_file(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as f:
        f.write(content.strip() + "\n")


def gen_planner(subdir, cls, init_fn, header, depth, ctor, ctor_args, methods, parent="ob::Planner",
                clear=None, setup=None, class_doc=None):
    path = os.path.join(BASE, "planners", subdir, f"{cls}.cpp")
    extra = "\n        ".join(fmt_method(cls, m) for m in methods)
    clear_expr = clear or f"&og::{cls}::clear"
    setup_expr = setup or f"&og::{cls}::setup"
    args = [a.strip() for a in ctor_args.split(",")]
    ctor_lines = f"        .def({ctor}"
    for a in args:
        parts = a.split("=")
        arg_name = parts[0].strip()
        if len(parts) > 1:
            ctor_lines += f', nb::arg("{arg_name}") = {parts[1].strip()}'
        else:
            ctor_lines += f', nb::arg("{arg_name}")'
    ctor_lines += ")"
    doc_arg = f', "{class_doc}"' if class_doc else ""
    content = f"""#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "{header}"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "{rel_init(depth)}"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::{init_fn}(nb::module_ &m)
{{
    nb::class_<og::{cls}, {parent}>(m, "{cls}"{doc_arg})
{ctor_lines}
{solve_lambda(cls)}
{planner_data(cls)}
        .def("clear", {clear_expr})
        .def("setup", {setup_expr})"""
    if extra:
        content += f"\n        {extra}"
    content += "\n}"
    write_file(path, content)
    return path


created = []

planners = [
    ("est", "EST", "initPlannersEst_EST", "ompl/geometric/planners/est/EST.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",)]),
    ("est", "BiEST", "initPlannersEst_BiEST", "ompl/geometric/planners/est/BiEST.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setRange", "distance"), ("getRange",)]),
    ("est", "ProjEST", "initPlannersEst_ProjEST", "ompl/geometric/planners/est/ProjEST.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
      ("setProjectionEvaluator", "projectionEvaluator",
       "static_cast<void (og::ProjEST::*)(const ob::ProjectionEvaluatorPtr &)>(&og::ProjEST::setProjectionEvaluator)"),
      ("setProjectionEvaluator", "name",
       "static_cast<void (og::ProjEST::*)(const std::string &)>(&og::ProjEST::setProjectionEvaluator)"),
      ("getProjectionEvaluator",)]),
    ("sst", "SST", "initPlannersSst_SST", "ompl/geometric/planners/sst/SST.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
      ("setSelectionRadius", "selectionRadius"), ("getSelectionRadius",),
      ("setPruningRadius", "pruningRadius"), ("getPruningRadius",)]),
    ("sbl", "SBL", "initPlannersSbl_SBL", "ompl/geometric/planners/sbl/SBL.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setProjectionEvaluator", "projectionEvaluator",
       "static_cast<void (og::SBL::*)(const ob::ProjectionEvaluatorPtr &)>(&og::SBL::setProjectionEvaluator)"),
      ("setProjectionEvaluator", "name",
       "static_cast<void (og::SBL::*)(const std::string &)>(&og::SBL::setProjectionEvaluator)"),
      ("getProjectionEvaluator",), ("setRange", "distance"), ("getRange",)]),
    ("pdst", "PDST", "initPlannersPdst_PDST", "ompl/geometric/planners/pdst/PDST.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setProjectionEvaluator", "projectionEvaluator",
       "static_cast<void (og::PDST::*)(const ob::ProjectionEvaluatorPtr &)>(&og::PDST::setProjectionEvaluator)"),
      ("setProjectionEvaluator", "name",
       "static_cast<void (og::PDST::*)(const std::string &)>(&og::PDST::setProjectionEvaluator)"),
      ("getProjectionEvaluator",), ("setGoalBias", "goalBias"), ("getGoalBias",)]),
    ("stride", "STRIDE", "initPlannersStride_STRIDE", "ompl/geometric/planners/stride/STRIDE.h", 2,
     "nb::init<const ob::SpaceInformationPtr &, bool, unsigned int, unsigned int, unsigned int, unsigned int, double>()",
     "si, useProjectedDistance=false, degree=16, minDegree=12, maxDegree=18, maxNumPtsPerLeaf=6, estimatedDimension=0.0",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setUseProjectedDistance", "useProjectedDistance"),
      ("getUseProjectedDistance",), ("setDegree", "degree"), ("getDegree",), ("setMinDegree", "minDegree"),
      ("getMinDegree",), ("setMaxDegree", "maxDegree"), ("getMaxDegree",),
      ("setMaxNumPtsPerLeaf", "maxNumPtsPerLeaf"), ("getMaxNumPtsPerLeaf",),
      ("setEstimatedDimension", "estimatedDimension"), ("getEstimatedDimension",),
      ("setRange", "distance"), ("getRange",), ("setMinValidPathFraction", "fraction"), ("getMinValidPathFraction",),
      ("setProjectionEvaluator", "projectionEvaluator",
       "static_cast<void (og::STRIDE::*)(const ob::ProjectionEvaluatorPtr &)>(&og::STRIDE::setProjectionEvaluator)"),
      ("setProjectionEvaluator", "name",
       "static_cast<void (og::STRIDE::*)(const std::string &)>(&og::STRIDE::setProjectionEvaluator)"),
      ("getProjectionEvaluator",)]),
    ("rrt", "LazyRRT", "initPlannersRrt_LazyRRT", "ompl/geometric/planners/rrt/LazyRRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",)]),
    ("rrt", "LBTRRT", "initPlannersRrt_LBTRRT", "ompl/geometric/planners/rrt/LBTRRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
      ("setApproximationFactor", "epsilon"), ("getApproximationFactor",)]),
    ("rrt", "LazyLBTRRT", "initPlannersRrt_LazyLBTRRT", "ompl/geometric/planners/rrt/LazyLBTRRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
      ("setApproximationFactor", "epsilon"), ("getApproximationFactor",)]),
    ("rrt", "TRRT", "initPlannersRrt_TRRT", "ompl/geometric/planners/rrt/TRRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
      ("setTempChangeFactor", "factor"), ("getTempChangeFactor",), ("setCostThreshold", "maxCost"), ("getCostThreshold",),
      ("setInitTemperature", "initTemperature"), ("getInitTemperature",),
      ("setFrontierThreshold", "frontier_threshold"), ("getFrontierThreshold",),
      ("setFrontierNodeRatio", "frontierNodeRatio"), ("getFrontierNodeRatio",)]),
    ("rrt", "ATRRT", "initPlannersRrt_ATRRT", "ompl/geometric/planners/rrt/ATRRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
      ("setTempChangeFactor", "factor"), ("getTempChangeFactor",), ("setCostThreshold", "maxCost"), ("getCostThreshold",),
      ("setInitTemperature", "initTemperature"), ("getInitTemperature",),
      ("setFrontierThreshold", "frontier_threshold"), ("getFrontierThreshold",),
      ("setFrontierNodeRatio", "frontierNodeRatio"), ("getFrontierNodeRatio",)]),
    ("rrt", "BiTRRT", "initPlannersRrt_BiTRRT", "ompl/geometric/planners/rrt/BiTRRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setRange", "distance"), ("getRange",), ("setTempChangeFactor", "factor"), ("getTempChangeFactor",),
      ("setCostThreshold", "maxCost"), ("getCostThreshold",), ("setInitTemperature", "initTemperature"),
      ("getInitTemperature",), ("setFrontierThreshold", "frontierThreshold"), ("getFrontierThreshold",),
      ("setFrontierNodeRatio", "frontierNodeRatio"), ("getFrontierNodeRatio",)]),
    ("rrt", "STRRTstar", "initPlannersRrt_STRRTstar", "ompl/geometric/planners/rrt/STRRTstar.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setRange", "distance"), ("getRange",), ("getOptimumApproxFactor",), ("setOptimumApproxFactor", "optimumApproxFactor"),
      ("getRewiringState",), ("setRewiringToOff",), ("setRewiringToRadius",), ("setRewiringToKNearest",),
      ("getRewireFactor",), ("setRewireFactor", "v"), ("getBatchSize",), ("setBatchSize", "v"),
      ("setTimeBoundFactorIncrease", "f"), ("setInitialTimeBoundFactor", "f"), ("setSampleUniformForUnboundedTime", "uniform")]),
    ("rlrt", "RLRT", "initPlannersRlrt_RLRT", "ompl/geometric/planners/rlrt/RLRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
      ("getKeepLast",), ("setKeepLast", "keepLast")]),
    ("rlrt", "BiRLRT", "initPlannersRlrt_BiRLRT", "ompl/geometric/planners/rlrt/BiRLRT.h", 2,
     "nb::init<const ob::SpaceInformationPtr &>()", "si",
     [("setRange", "distance"), ("getRange",), ("setMaxDistanceNear", "dNear"), ("getMaxDistanceNear",),
      ("getKeepLast",), ("setKeepLast", "keepLast")]),
]

for p in planners:
    created.append(gen_planner(*p))

# RRTsharp inherits RRTXstatic
created.append(gen_planner(
    "rrt", "RRTsharp", "initPlannersRrt_RRTsharp", "ompl/geometric/planners/rrt/RRTsharp.h", 2,
    "nb::init<const ob::SpaceInformationPtr &>()", "si", [], parent="og::RRTXstatic",
    clear="&og::RRTXstatic::clear", setup="&og::RRTsharp::setup"))

# RRTXstatic - many methods
rrtx_methods = [
    ("setGoalBias", "goalBias"), ("getGoalBias",), ("setInformedSampling", "informedSampling"), ("getInformedSampling",),
    ("setSampleRejection", "reject"), ("getSampleRejection",), ("setNumSamplingAttempts", "numAttempts"),
    ("getNumSamplingAttempts",), ("setEpsilon", "epsilon"), ("getEpsilon",), ("setRange", "distance"), ("getRange",),
    ("setRewireFactor", "rewireFactor"), ("getRewireFactor",), ("setKNearest", "useKNearest"), ("getKNearest",),
    ("setUpdateChildren", "val"), ("getUpdateChildren",), ("setVariant", "variant"), ("getVariant",),
    ("setAlpha", "a"), ("getAlpha",), ("numIterations",), ("bestCost",),
]
created.append(gen_planner(
    "rrt", "RRTXstatic", "initPlannersRrt_RRTXstatic", "ompl/geometric/planners/rrt/RRTXstatic.h", 2,
    "nb::init<const ob::SpaceInformationPtr &>()", "si", rrtx_methods))

# TRRTstar - similar to RRTstar + temp methods
trrtstar_methods = [
    ("setGoalBias", "goalBias"), ("getGoalBias",), ("setRange", "distance"), ("getRange",),
    ("setRewireFactor", "rewireFactor"), ("getRewireFactor",), ("setDelayCC", "delayCC"), ("getDelayCC",),
    ("setTreePruning", "prune"), ("getTreePruning",), ("setPruneThreshold", "pp"), ("getPruneThreshold",),
    ("setAdmissibleCostToCome", "admissible"), ("getAdmissibleCostToCome",), ("setBatchSize", "batchSize"),
    ("getBatchSize",), ("setKNearest", "useKNearest"), ("getKNearest",), ("setNumSamplingAttempts", "numAttempts"),
    ("getNumSamplingAttempts",), ("numIterations",), ("bestCost",), ("setTempChangeFactor", "factor"),
    ("getTempChangeFactor",), ("setCostThreshold", "maxCost"), ("getCostThreshold",),
    ("setInitTemperature", "initTemperature"), ("getInitTemperature",),
]
created.append(gen_planner(
    "rrt", "TRRTstar", "initPlannersRrt_TRRTstar", "ompl/geometric/planners/rrt/TRRTstar.h", 2,
    "nb::init<const ob::SpaceInformationPtr &>()", "si", trrtstar_methods))

# LazyPRM with doc warning
lazyprm_doc = (
    "Warning: LazyPRM may use internal threads for solution checking. "
    "Avoid Python StateValidityChecker trampolines when multithreading is enabled."
)
path = os.path.join(BASE, "planners", "prm", "LazyPRM.cpp")
lazyprm_extra = """
        .def(nb::init<const ob::PlannerData &, bool>(), nb::arg("data"), nb::arg("starStrategy") = false)
        .def("setRange", &og::LazyPRM::setRange, nb::arg("distance"))
        .def("getRange", &og::LazyPRM::getRange)
        .def("setDefaultConnectionStrategy", &og::LazyPRM::setDefaultConnectionStrategy)
        .def("setMaxNearestNeighbors", &og::LazyPRM::setMaxNearestNeighbors, nb::arg("k"))
        .def("clearQuery", &og::LazyPRM::clearQuery)
        .def("clearValidity", &og::LazyPRM::clearValidity)
        .def("milestoneCount", &og::LazyPRM::milestoneCount)
        .def("edgeCount", &og::LazyPRM::edgeCount)"""
content = f"""#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "{rel_init(2)}"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_LazyPRM(nb::module_ &m)
{{
    nb::class_<og::LazyPRM, ob::Planner>(m, "LazyPRM", "{lazyprm_doc}")
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"), nb::arg("starStrategy") = false)
{lazyprm_extra}
{solve_lambda("LazyPRM")}
{planner_data("LazyPRM")}
        .def("clear", &og::LazyPRM::clear)
        .def("setup", &og::LazyPRM::setup)
}}
"""
write_file(path, content)
created.append(path)

# LazyPRMstar
created.append(gen_planner(
    "prm", "LazyPRMstar", "initPlannersPrm_LazyPRMstar", "ompl/geometric/planners/prm/LazyPRMstar.h", 2,
    "nb::init<const ob::SpaceInformationPtr &>()", "si", [],
    parent="og::LazyPRM", clear="&og::LazyPRM::clear", setup="&og::LazyPRM::setup"))
path = os.path.join(BASE, "planners", "prm", "LazyPRMstar.cpp")
content = open(path).read() + "\n        .def(nb::init<const ob::PlannerData &>(), nb::arg(\"data\"))\n}"
write_file(path, content[:-2] + "\n        .def(nb::init<const ob::PlannerData &>(), nb::arg(\"data\"))\n}\n")

# SPARS
spars_doc = (
    "Warning: SPARS may use internal threads for solution checking. "
    "Avoid Python StateValidityChecker trampolines when multithreading is enabled."
)
path = os.path.join(BASE, "planners", "prm", "SPARS.cpp")
spars_extra = """
        .def("setMaxFailures", &og::SPARS::setMaxFailures, nb::arg("m"))
        .def("getMaxFailures", &og::SPARS::getMaxFailures)
        .def("setDenseDeltaFraction", &og::SPARS::setDenseDeltaFraction, nb::arg("d"))
        .def("setSparseDeltaFraction", &og::SPARS::setSparseDeltaFraction, nb::arg("d"))
        .def("setStretchFactor", &og::SPARS::setStretchFactor, nb::arg("t"))
        .def("getDenseDeltaFraction", &og::SPARS::getDenseDeltaFraction)
        .def("getSparseDeltaFraction", &og::SPARS::getSparseDeltaFraction)
        .def("getStretchFactor", &og::SPARS::getStretchFactor)
        .def("clearQuery", &og::SPARS::clearQuery)
        .def("milestoneCount", &og::SPARS::milestoneCount)
        .def("guardCount", &og::SPARS::guardCount)"""
content = f"""#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "{rel_init(2)}"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARS(nb::module_ &m)
{{
    nb::class_<og::SPARS, ob::Planner>(m, "SPARS", "{spars_doc}")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
{construct_roadmap_lambda("SPARS", stop_arg=True)}
{solve_lambda("SPARS")}
{planner_data("SPARS")}
        .def("clear", &og::SPARS::clear)
        .def("setup", &og::SPARS::setup)
{spars_extra}
}}
"""
write_file(path, content)
created.append(path)

# SPARStwo
sparstwo_doc = (
    "Warning: SPARStwo may use internal threads for solution checking. "
    "Avoid Python StateValidityChecker trampolines when multithreading is enabled."
)
path = os.path.join(BASE, "planners", "prm", "SPARStwo.cpp")
sparstwo_extra = """
        .def("setStretchFactor", &og::SPARStwo::setStretchFactor, nb::arg("t"))
        .def("setSparseDeltaFraction", &og::SPARStwo::setSparseDeltaFraction, nb::arg("D"))
        .def("setDenseDeltaFraction", &og::SPARStwo::setDenseDeltaFraction, nb::arg("d"))
        .def("setMaxFailures", &og::SPARStwo::setMaxFailures, nb::arg("m"))
        .def("getMaxFailures", &og::SPARStwo::getMaxFailures)
        .def("getDenseDeltaFraction", &og::SPARStwo::getDenseDeltaFraction)
        .def("getSparseDeltaFraction", &og::SPARStwo::getSparseDeltaFraction)
        .def("getStretchFactor", &og::SPARStwo::getStretchFactor)
        .def("clearQuery", &og::SPARStwo::clearQuery)
        .def("milestoneCount", &og::SPARStwo::milestoneCount)"""
content = f"""#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "{rel_init(2)}"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersPrm_SPARStwo(nb::module_ &m)
{{
    nb::class_<og::SPARStwo, ob::Planner>(m, "SPARStwo", "{sparstwo_doc}")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
{construct_roadmap_lambda("SPARStwo", stop_arg=True)}
{solve_lambda("SPARStwo")}
{planner_data("SPARStwo")}
        .def("clear", &og::SPARStwo::clear)
        .def("setup", &og::SPARStwo::setup)
{sparstwo_extra}
}}
"""
write_file(path, content)
created.append(path)

print(f"Generated {len(created)} planner files")
