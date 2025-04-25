#!/usr/bin/env python3
import time
from math import sin, cos
from ompl import base as ob
from ompl import control as oc
try:
    import numpy as np
    import pybullet as p
    import pybullet_data
except ImportError:
    print("Error: Please install the required packages.")
    exit(1)

def setup_pybullet(mode=p.DIRECT):
    """Start PyBullet, load a plane, two box obstacles, and a box‐car."""
    p.connect(mode)
    p.setGravity(0, 0, -9.0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # Create box obstacles
    box_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,0.2,0.2])
    obs0 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col,
                             basePosition=[ 0.5,  0.0, 0.2])
    obs1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col,
                             basePosition=[-0.3,  0.4, 0.2])

    # Create a "car" as a box (for collision & visualization)
    car_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,0.1,0.05])
    car_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2,0.1,0.05],
                                  rgbaColor=[0.0,0.0,1.0,1.0])
    car = p.createMultiBody(baseMass=1,
                            baseCollisionShapeIndex=car_col,
                            baseVisualShapeIndex=car_vis,
                            basePosition=[0,0,0.05])
    return car, [obs0, obs1]

def make_state_validity_checker(car_id, obstacles, joint_indices=None):
    def is_valid(si, state):
        if si.satisfiesBounds(state) is not True:
            return False
        x, y, yaw = state.getX(), state.getY(), state.getYaw()
        pos = [x, y, 0.05]
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        p.resetBasePositionAndOrientation(car_id, pos, orn)
        p.stepSimulation()
        # Check collision with any obstacle
        for obs in obstacles:
            if p.getContactPoints(bodyA=car_id, bodyB=obs):
                return False
        return True
    return is_valid

def propagate(start, control, duration, result):
    """Simple unicycle model Euler integration."""
    v, omega = control[0], control[1]
    theta = start.getYaw()
    result.setX(start.getX() + v * cos(theta) * duration)
    result.setY(start.getY() + v * sin(theta) * duration)
    result.setYaw(theta + omega * duration)

def plan_control_rrt():
    # 1) State space = SE2
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1); bounds.setHigh(1)
    space.setBounds(bounds)

    # 2) Control space = (v, ω) ∈ [-1,1]^2
    cspace = oc.RealVectorControlSpace(space, 2)
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-1); cbounds.setHigh(1)
    cspace.setBounds(cbounds)

    # 3) SpaceInformation
    si = oc.SpaceInformation(space, cspace)
    si.setPropagationStepSize(0.05)

    # 4) SimpleSetup
    ss = oc.SimpleSetup(si)
    ss.setStateValidityChecker(lambda s: is_valid_fn(si, s))
    ss.setStatePropagator(propagate)

    # 5) Start & goal
    start = space.allocState()
    goal  = space.allocState()
    start.setX(-0.8); start.setY(-0.8); start.setYaw(0)
    goal .setX( 0.8); goal .setY( 0.8); goal .setYaw(0)
    ss.setStartAndGoalStates(start, goal, 0.1)

    # 6) RRT planner
    planner = oc.RRT(si)
    planner.setGoalBias(0.1)
    ss.setPlanner(planner)

    # 7) Solve for up to 10 seconds
    print("Planning (DIRECT mode)…")
    solved = ss.solve(10.0)
    if not solved:
        print("No solution found.")
        return None
    path = ss.getSolutionPath()
    path.interpolate() 
    print("✔ Found solution!")
    print("  # states:", path.getStateCount())
    return path

if __name__ == "__main__":
    # --- PLAN in headless mode ---
    car_id, obstacles = setup_pybullet(mode=p.DIRECT)
    is_valid_fn = make_state_validity_checker(car_id, obstacles)
    path = plan_control_rrt()
    p.disconnect()

    if path is None:
        exit(1)

    # --- VISUALIZE in GUI mode ---
    car_id, obstacles = setup_pybullet(mode=p.GUI)
    print("Replaying in GUI. Ctrl+C to exit.")
    while True:
        for i in range(path.getStateCount()):
            st = path.getState(i)
            pos = [st.getX(), st.getY(), 0.05]
            orn = p.getQuaternionFromEuler([0,0,st.getYaw()])
            p.resetBasePositionAndOrientation(car_id, pos, orn)
            p.stepSimulation()
            time.sleep(0.05)
