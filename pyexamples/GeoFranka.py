import time
from ompl import base as ob
from ompl import geometric as og

try:
    import numpy as np
    import pybullet as p
    import pybullet_data
except ImportError:
    print("Error: Please install the required packages.")
    exit(1)
    
def setup_pybullet(mode=p.DIRECT):
    """Start PyBullet in the given mode, load plane, Panda, and two box obstacles."""
    p.connect(mode)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Ground plane
    p.loadURDF("plane.urdf")

    # Panda arm, fixed base
    panda = p.loadURDF(
        "franka_panda/panda.urdf",
        basePosition=[0, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True
    )

    # Two box obstacles
    box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
    obs0 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col,
                             basePosition=[0.5,  0.0, 0.2])
    obs1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col,
                             basePosition=[0.3,  0.3, 1.0])

    return panda, [obs0, obs1]

def get_revolute_joint_indices(robot_id):
    joints = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] == p.JOINT_REVOLUTE:
            joints.append(i)
    return joints

def get_joint_limits(robot_id, joint_indices):
    lowers, uppers = [], []
    for j in joint_indices:
        info = p.getJointInfo(robot_id, j)
        lowers.append(info[8])
        uppers.append(info[9])
    return lowers, uppers

def make_state_validity_checker(robot_id, obs_list, joint_indices):
    def is_valid(state):
        # Apply the joint angles
        for idx, angle in zip(joint_indices, state):
            p.resetJointState(robot_id, idx, angle)
        # Step once to register collisions
        p.stepSimulation()
        # Check contacts with each obstacle
        for obs in obs_list:
            if p.getContactPoints(bodyA=robot_id, bodyB=obs):
                return False
        return True
    return is_valid

def plan_with_rrt(robot_id, obs_list, joint_indices, lowers, uppers):
    dim = len(joint_indices)
    # 1) Joint‐space
    space = ob.RealVectorStateSpace(dim)
    bounds = ob.RealVectorBounds(dim)
    for i in range(dim):
        bounds.setLow(i, lowers[i])
        bounds.setHigh(i, uppers[i])
    space.setBounds(bounds)

    # 2) SpaceInformation
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(make_state_validity_checker(robot_id, obs_list, joint_indices))
    si.setup()

    # 3) Start & goal
    start = si.allocState()
    goal  = si.allocState()
    # start at zeros, goal at some demo config
    demo_goal = [0.3207, 0.5475, 0.2826, -1.0339, -0.3758, 1.4597, 0.0110]
    for i in range(dim):
        start[i] = 0.0
        goal[i]  = demo_goal[i]

    # 4) RRT planner
    rrt = og.RRT(si)
    rrt.setGoalBias(0.1)
    rrt.setRange(0.5)

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(rrt)

    # 5) Solve with a 20 s cap
    print(f"Planning in DIRECT mode (no GUI)…")
    t0 = time.time()
    solved = ss.solve(ob.PlannerTerminationCondition(lambda: time.time() - t0 > 20))
    path = None
    if solved:
        path = ss.getSolutionPath()
        path.interpolate()  # densify
        if solved.StatusType == ob.EXACT_SOLUTION:
            print("✓ Exact solution found.")
        else:
            print("✓ Approximate solution found.")
        print("  Path length:    ", path.length())
        print("  Num of states:  ", path.getStateCount())
    else:
        print("✘ No solution found within 20 s")
    return path
if __name__ == "__main__":
    # 1) PLAN in DIRECT mode
    panda_id, obstacles = setup_pybullet(mode=p.DIRECT)
    joint_idxs = get_revolute_joint_indices(panda_id)
    low, high = get_joint_limits(panda_id, joint_idxs)
    solution_path = plan_with_rrt(panda_id, obstacles, joint_idxs, low, high)

    # Clean up the DIRECT connection
    p.disconnect()

    # 2) VISUALIZE in GUI mode
    if solution_path is not None:
        panda_id, obstacles = setup_pybullet(mode=p.GUI)
        # Play back each state
        print("Replaying path in GUI…")
        while True:
            for state in solution_path.getStates():
                for idx, angle in zip(joint_idxs, state):
                    p.resetJointState(panda_id, idx, angle)
                p.stepSimulation()
                time.sleep(0.1)
    else:
        print("Nothing to show.")
