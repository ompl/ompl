import pytest
import tempfile
import os

from ompl import base as ob
from ompl import geometric as og

from geo_env import create_simple_setup, solve_with_planner


def test_planner_data_basic():
    """Test basic PlannerData functionality."""
    # Create a SimpleSetup and solve
    ss = create_simple_setup()
    si = ss.getSpaceInformation()
    
    # Solve with RRTConnect
    rrt_planner = og.RRTConnect(si)
    solution_path = solve_with_planner(ss, rrt_planner)
    assert solution_path is not None
    
    # Get planner data
    pd = ob.PlannerData(si)
    ss.getPlanner().getPlannerData(pd)
    
    # Check basic properties
    num_vertices = pd.numVertices()
    num_edges = pd.numEdges()
    print(f"Vertices: {num_vertices}, Edges: {num_edges}")
    
    assert num_vertices > 0, "PlannerData should have vertices"
    assert num_edges >= 0, "PlannerData should have edges"
    
    # Check start and goal vertices
    num_starts = pd.numStartVertices()
    num_goals = pd.numGoalVertices()
    print(f"Start vertices: {num_starts}, Goal vertices: {num_goals}")
    assert num_starts > 0, "Should have at least one start vertex"
    
    # Test vertex access
    if num_vertices > 0:
        vertex = pd.getVertex(0)
        state = vertex.getState()
        assert state is not None
    
    # Test edge access
    edges = pd.getEdges(0)
    print(f"Edges from vertex 0: {edges}")
    
    # Test output methods
    graphml = pd.printGraphML()
    assert len(graphml) > 0, "GraphML output should not be empty"
    print(f"GraphML length: {len(graphml)}")
    
    graphviz = pd.printGraphviz()
    assert len(graphviz) > 0, "Graphviz output should not be empty"


def test_planner_data_storage():
    """Test PlannerDataStorage save and load."""
    # Create a SimpleSetup and solve
    ss = create_simple_setup()
    si = ss.getSpaceInformation()
    
    # Solve with RRTConnect
    rrt_planner = og.RRTConnect(si)
    solution_path = solve_with_planner(ss, rrt_planner)
    assert solution_path is not None
    
    # Get planner data
    pd = ob.PlannerData(si)
    ss.getPlanner().getPlannerData(pd)
    
    original_vertices = pd.numVertices()
    original_edges = pd.numEdges()
    print(f"Original: {original_vertices} vertices, {original_edges} edges")
    
    # Create a temporary file for storage
    with tempfile.NamedTemporaryFile(suffix='.pd', delete=False) as f:
        temp_filename = f.name
    
    try:
        # Store planner data
        storage = ob.PlannerDataStorage()
        result = storage.store(pd, temp_filename)
        assert result, "Failed to store PlannerData"
        assert os.path.exists(temp_filename), "File should exist after store"
        
        # Load into new PlannerData
        pd2 = ob.PlannerData(si)
        result = storage.load(temp_filename, pd2)
        assert result, "Failed to load PlannerData"
        
        # Verify loaded data matches original
        loaded_vertices = pd2.numVertices()
        loaded_edges = pd2.numEdges()
        print(f"Loaded: {loaded_vertices} vertices, {loaded_edges} edges")
        
        assert loaded_vertices == original_vertices, "Vertex count should match"
        assert loaded_edges == original_edges, "Edge count should match"
        
    finally:
        # Clean up
        if os.path.exists(temp_filename):
            os.remove(temp_filename)


def test_planner_data_vertex():
    """Test PlannerDataVertex functionality."""
    ss = create_simple_setup()
    si = ss.getSpaceInformation()
    space = si.getStateSpace()
    
    # Allocate a state
    state = space.allocState()
    
    # Create a vertex with a tag
    vertex = ob.PlannerDataVertex(state, 42)
    assert vertex.getTag() == 42
    
    # Change the tag
    vertex.setTag(100)
    assert vertex.getTag() == 100
    
    # Get the state back
    retrieved_state = vertex.getState()
    assert retrieved_state is not None
    
if __name__ == "__main__":
    test_planner_data_basic()
    test_planner_data_storage()
    test_planner_data_vertex()