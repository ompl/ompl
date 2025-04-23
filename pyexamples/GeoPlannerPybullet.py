import time
import numpy as np
import pybullet as p
import pybullet_data
from ompl import base as ob
from ompl import geometric as og

def setup_pybullet():
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Ground plane
    p.loadURDF("plane.urdf")

    # Panda arm, fixed base
    panda = p.loadURDF("franka_panda/panda.urdf",
                        basePosition=[0, 0, 0],
                        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=True)

    # Simple box obstacles
    box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
    obs0 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col, basePosition=[0.5,  0.0, 0.2])
    obs1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col, basePosition=[0.3, 0.3, 1])

    return panda, [obs0, obs1]

def get_revolute_joint_indices(robot_id):
    """Return list of joint indices that are revolute (the 7 Panda joints)."""
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
        lowers.append(info[8])   # joint lower limit
        uppers.append(info[9])   # joint upper limit
    return lowers, uppers

def make_state_validity_checker(robot_id, obs_list, joint_indices):
    def is_valid(state):
        # Apply the joint angles
        for idx, angle in zip(joint_indices, state):
            p.resetJointState(robot_id, idx, angle)
        p.stepSimulation()
        # Query for any contact involving the robot
        for obs in obs_list:
            contacts = p.getContactPoints(bodyA=robot_id, bodyB=obs)
            if len(contacts) > 0:
                return False
        return True
    return is_valid

def plan_with_rrt(robot_id, obs_list, joint_indices, lowers, uppers):
    # 1) Define OMPL joint‐space
    dim = len(joint_indices)
    space = ob.RealVectorStateSpace(dim)
    bounds = ob.RealVectorBounds(dim)
    for i in range(dim):
        bounds.setLow(i, lowers[i])
        bounds.setHigh(i, uppers[i])
    space.setBounds(bounds)

    # 2) SpaceInformation & validity checker
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(make_state_validity_checker(robot_id, obs_list, joint_indices))
    si.setup()

    # 3) Start & goal
    start = si.allocState()
    goal  = si.allocState()
    # Example: start at all zeros, goal at mid‐range
    start_config = np.zeros(dim)
    goal_config  = np.array([0.3207, 0.5475, 0.2826, -1.0339, -0.3758, 1.4597, 0.0110])
    for i in range(dim):
        start[i] = start_config[i]
        goal[i]  = goal_config[i]

    # 4) Planner (RRT)
    rrt = og.RRT(si)
    rrt.setGoalBias(0.1)
    rrt.setRange(0.5)

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(rrt)

    start_time = time.time()
    ptc = ob.PlannerTerminationCondition(
        lambda: (time.time() - start_time) > 20
    )

    # 5) Solve with 5s timeout
    print(f"Solving RRT in {dim}-D joint space…")
    solved = ss.solve(ptc)
    path = None
    if solved.StatusType == ob.EXACT_SOLUTION:
        path = ss.getSolutionPath()
        path.interpolate()
        print("  Path length:", path.length())
        print("  Number of states:", path.getStateCount())
    return solved, path

if __name__ == "__main__":
    panda_id, obs_list = setup_pybullet()
    joint_idxs = get_revolute_joint_indices(panda_id)
    lowers, uppers = get_joint_limits(panda_id, joint_idxs)

    solved, plan = plan_with_rrt(panda_id, obs_list, joint_idxs, lowers, uppers)

    if solved.StatusType == ob.EXACT_SOLUTION:
        while True:
        # Visualize the path in PyBullet
            for state in plan.getStates():
                for idx, angle in zip(joint_idxs, state):
                    p.resetJointState(panda_id, idx, angle)
                p.stepSimulation()
                time.sleep(0.1)
    else:
        print("No valid path found.")
