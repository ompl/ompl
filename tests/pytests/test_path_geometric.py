from ompl import geometric as og

from geo_env import create_simple_setup, solve_with_planner


def test_path_geometric_check_and_repair():
    ss = create_simple_setup()
    planner = og.RRTConnect(ss.getSpaceInformation())
    path = solve_with_planner(ss, planner, timeout=2.0)
    assert path is not None

    result = path.checkAndRepair(100)
    assert isinstance(result, tuple)
    assert len(result) == 2
    assert isinstance(result[0], bool)
    assert isinstance(result[1], bool)
