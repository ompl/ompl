from ompl import base as ob


def test_cost_convergence_process_new_solution():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(lambda _state: True)
    si.setup()

    pdef = ob.ProblemDefinition(si)
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))

    ptc = ob.CostConvergenceTerminationCondition(pdef, 5, 1.0)
    assert not ptc()

    for i in range(10):
        ptc.processNewSolution(ob.Cost(10.0))
        if i < 4:
            assert not ptc()
        else:
            assert ptc()
