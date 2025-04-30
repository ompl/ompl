# class MyControlRRT(ob.Planner):
#     def __init__(self, si, name):
#         super().__init__(si, name)
#         self._sampler = si.allocStateSampler()
#         self._goal_bias = 0.05

#     class TreeNode:
#         def __init__(self, cur_state:ob.State, cur_control:oc.Control, parent_node):
#             self.value = cur_state
#             self.parent_node = parent_node
#             self.children_nodes = []
#             if parent_node is not None:
#                 parent_node.children_nodes.append(self)
#     class Tree:
#         def __init__(self, root_state, si):
#             self.root = MyRRT.TreeNode(root_state, None)
#             self.nodes = [self.root]
#             self.si = si
#         def add_state(self, cur_state, parent_node):
#             new_node = MyRRT.TreeNode(cur_state, parent_node)
#             self.nodes.append(new_node)
#             return new_node
#         def find_state(self, state, from_node = None):
#             if from_node is None:
#                 from_node = self.root
#             if from_node.value == state:
#                 return from_node
#             for child in from_node.children_nodes:
#                 return self.find_state(state, child)
#             return None
#         def find_nearest(self, state):
#             nearest_node = None
#             nearest_dis = float('inf')
#             for node in self.nodes:
#                 dis = self.si.distance(node.value, state)
#                 if dis < nearest_dis:
#                     nearest_dis = dis
#                     nearest_node = node
#             return nearest_node

#         def find_random(self):
#             return random.choice(self.nodes)
#         def backtrace(self, node):
#             path = []
#             while node:
#                 path.append(node.value)
#                 node = node.parent_node
#             return path
        
#     def solve(self, ptc):
#         pdef = self.getProblemDefinition()
#         goal = pdef.getGoal()
#         can_sample = hasattr(goal, 'sampleGoal')
#         if not can_sample:
#             raise Exception("Unable to sample goal!")
#         si = self.getSpaceInformation()
#         pi = self.getPlannerInputStates()
#         sic = self.getControlSpace()
#         st = pi.nextStart()
#         if not st:
#             raise Exception("No start state!")
#         # Initialize the tree
#         tree = MyRRT.Tree(st, si)
#         solution = None
#         solved = False
#         goal_node = None
#         while not ptc():
#             new_state = si.allocState()
#             if random.random() < self._goal_bias:
#                 goal.sampleGoal(new_state)
#             else:
#                 self._sampler.sampleUniform(new_state)
#             nearest_node = tree.find_nearest(new_state)
#             # print(nearest_node.value)
#             if si.checkMotion(nearest_node.value, new_state):
#                 new_node = tree.add_state(new_state, nearest_node)
#                 dis_to_goal = goal.distanceGoal(new_state)
#                 if goal.isSatisfied(new_state):
#                     solved = True
#                     goal_node = new_node
#                     break
#         if solved:
#             path = og.PathGeometric(si)
#             path_states = tree.backtrace(goal_node)
#             for s in path_states:
#                 path.append(s)
#             pdef.addSolutionPath(path)
#         return ob.PlannerStatus(solved, True)
#     def setup(*args, **argv):
#         pass

# def isStateValid(spaceInformation, state):
#     # perform collision checking or check if other constraints are satisfied
#     return spaceInformation.satisfiesBounds(state)

# def propagate(temp1, control, duration, state):
#     # For demonstration, intentionally messing up the partial usage
#     # but let's keep it as your snippet.  Real usage would typically do cos/sin.
#     state.setX(temp1.getX() + control[0] * duration * (temp1.getYaw()))
#     state.setY(temp1.getY() + control[0] * duration * (temp1.getYaw()))
#     state.setYaw(temp1.getYaw() + control[1] * duration)

# def test_customize_control_planner():
#     # 1) Construct the SE2 state space
#     space = ob.SE2StateSpace()
    
#     # set R^2 bounds
#     bounds = ob.RealVectorBounds(2)
#     bounds.setLow(-1)
#     bounds.setHigh(1)
#     space.setBounds(bounds)

#     # 2) Create a real-vector control space of dimension=2
#     cspace = oc.RealVectorControlSpace(space, 2)
#     cbounds = ob.RealVectorBounds(2)
#     cbounds.setLow(-0.3)
#     cbounds.setHigh(0.3)
#     cspace.setBounds(cbounds)

#     # 3) Construct a SpaceInformation from that (space, cspace)
#     si = oc.SpaceInformation(space, cspace)
#     si.setPropagationStepSize(1.0)

#     # 4) Build a SimpleSetup from the SpaceInformation
#     ss = oc.SimpleSetup(si)

#     # 5) Provide a state validity checker as a lambda
#     # This partial-lambda structure ensures the argument signature matches (State*) -> bool
#     ss.setStateValidityChecker(lambda s: isStateValid(ss.getSpaceInformation(), s))
    
#     # 6) Provide a state propagator
#     ss.setStatePropagator(propagate)

#     # 7) Create start and goal states
#     start = space.allocState()
#     start.setX(-0.5)
#     start.setY(0.0)
#     start.setYaw(0.0)

#     goal = space.allocState()
#     goal.setX(0.0)
#     goal.setY(0.5)
#     goal.setYaw(0.0)

#     # 8) Set the start and goal states
#     ss.setStartAndGoalStates(start, goal, 0.05)

#     planner = MyControlRRT(si, "MyControlRRT")
#     ss.setPlanner(planner)
#     # 9) Attempt to solve
#     solved = ss.solve(2)

#     # If solved, optionally retrieve path
#     if solved:
#         print("Found solution path.")
#         path = ss.getSolutionPath()
#         path.printAsMatrix()

#     del ss
#     import gc
#     gc.collect()
