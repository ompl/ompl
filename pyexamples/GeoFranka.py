import time

import sys
sys.path.insert(0, "/home/wg/Documents/ompl_test_env/ompl/build/bindings/")

from ompl import base as ob
from ompl import geometric as og

try:
    import numpy as np
    import pybullet
    import pybullet_data
    from pybullet_utils import bullet_client
except ImportError:
    print("Error: Please install the required packages.")
    exit(1)
    

class PyBulletEnv:
    def __init__(self, gui:bool = True):
        self.gui = gui
        self.create_env()
        self.dim = 7

    def create_env(self):
        self.env = bullet_client.BulletClient(connection_mode=pybullet.GUI if self.gui else pybullet.DIRECT)
        self.env.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.env.loadURDF("plane.urdf")

        self.robot = self.env.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[0, 0, 0],
            baseOrientation=self.env.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )

        self.obs = []
        plane = self.env.loadURDF("plane.urdf")
        self.obs.append(plane)

        box_col = self.env.createCollisionShape(self.env.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
        obs0 = self.env.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col,
                                        basePosition=[0.5,  0.0, 0.2])
        obs1 = self.env.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col,
                                        basePosition=[0.3,  0.3, 1.0])
        self.obs.append(obs0)
        self.obs.append(obs1)

    def step(self):
        self.env.stepSimulation()

    def disconnect(self):
        self.env.disconnect()

    def get_joint_limits(self):
        lowers, uppers = [], []
       
        for j in range(self.dim):
            info = self.env.getJointInfo(self.robot, j)
            lowers.append(info[8])
            uppers.append(info[9])
        return np.array(lowers), np.array(uppers)
    
    def set_config(self, config):
        for j, angle in enumerate(config):
            self.env.resetJointState(self.robot, j, angle)
        self.env.stepSimulation()

    def get_config(self):
        config = []
        for j in range(self.dim):
            joint_angle = self.env.getJointState(self.robot, j)[0]
            config.append(joint_angle)
        return config
    
    def is_valid(self, config):
        self.set_config(config)

        # self collision check
        if self.env.getContactPoints(self.robot, self.robot):
            return False
        # obstacle collision check
        for obs in self.obs:
            if self.env.getContactPoints(bodyA=self.robot, bodyB=obs):
                return False
        return True

def state_to_list(state, num_arms):
    joint_angles = []
    for i in range(num_arms):
        joint_angles.append(state[i])
    return joint_angles

def make_state_validity_checker(env: PyBulletEnv):
    def is_valid(state):
        joint_angles = state_to_list(state, env.dim)
        return env.is_valid(joint_angles)
    return is_valid

def plan(env: PyBulletEnv, start, goal, planner_type='RRTConnect', time_limit=10.0, simplify=True):
    lowers, uppers = env.get_joint_limits()
    dim = env.dim

    space = ob.RealVectorStateSpace(dim)
    bounds = ob.RealVectorBounds(dim)
    for i in range(dim):
        bounds.setLow(i, lowers[i])
        bounds.setHigh(i, uppers[i])
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(make_state_validity_checker(env))


    start_state = si.allocState()
    goal_state = si.allocState()
    
    for i in range(dim):
        start_state[i] = start[i]
        goal_state[i] = goal[i]

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start_state, goal_state)

    if planner_type == 'RRT':
        planner = og.RRT(si)
    elif planner_type == 'RRTConnect':
        planner = og.RRTConnect(si)
    else:
        raise ValueError("Unsupported planner type")

    ss.setPlanner(planner)
    solved = ss.solve(time_limit)  # 10 seconds timeout
    if solved.StatusType != ob.EXACT_SOLUTION:
        print("No solution found")
        return None
    
    if simplify:
        ss.simplifySolution(1)

    path = ss.getSolutionPath()
    path.interpolate() 
    return path

if __name__ == "__main__":
    env = PyBulletEnv(gui=True)
    start_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal_config = [0.3207, 0.5475, 0.2826, -1.0339, -0.3758, 1.4597, 0.0110]

    path = plan(env, start_config, goal_config, planner_type='RRTConnect')

    if path:
        while True: 
            for state in path.getStates():
                joint_angles = state_to_list(state, env.dim)
                env.set_config(joint_angles)
                time.sleep(0.1)    
