import sys
import pytest
from ompl import base as ob
from ompl import geometric as og


def test_path_geometric_basic():
    """
    Test the PathGeometric binding in a simple RealVectorStateSpace (2D).
    We'll create states, append them to the path, test interpolation, length, etc.
    """

    # 1) Create a 2D RealVector state space with bounds [0,1] in both dimensions.
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # 2) Create a SpaceInformation for that space and set it up with trivial validity.
    si = ob.SpaceInformation(space)
    # (We won't define an actual collision checker here, so si.isValid() won't do much.)

    # 3) Allocate two or three states.
    st1 = space.allocState()
    st2 = space.allocState()
    st3 = space.allocState()

    # Manually set st1 to (0.25, 0.25).
    st1[0] = 0.25
    st1[1] = 0.25

    # st2, st3 can be random states.
    sampler = si.allocStateSampler()
    sampler.sampleUniform(st2)
    sampler.sampleUniform(st3)

    # 4) Create a PathGeometric and append states.
    path = og.PathGeometric(si)
    path.append(st1)
    path.append(st2)
    path.append(st3)

    # Check the state count.
    assert path.getStateCount() == 3, f"Expected 3 states in the path, got {path.getStateCount()}"

    # 5) Print the path as a string.
    path_str = path.print()
    print("Initial path:\n", path_str)

    # 6) Check the length.
    path_len = path.length()
    print("Path length before interpolation:", path_len)
    assert path_len >= 0.0, "Path length should be non-negative."

    # 7) Try interpolating the path with some extra points, then compare new length.
    path.interpolate(10)
    # We can't strictly predict the new length, but we can check that state count increased.
    assert path.getStateCount() >= 10, "After interpolating with 'count=10', state count should be >= 10."
    new_len = path.length()
    print("Path length after interpolation:", new_len)

    # 8) Try subdividing or reversing the path for demonstration.
    path.subdivide()
    path.reverse()
    print("Path after subdivide + reverse:\n", path.print())

    # 9) Call random() or randomValid() to randomize the path. (Will not do much if no validity checks.)
    path.random()
    print("Path after randomization:\n", path.print())

    # 10) Clear the path and confirm it is empty.
    path.clear()
    assert path.getStateCount() == 0, "After clearing, the path should have 0 states."
    print("Path is cleared. State count =", path.getStateCount())
