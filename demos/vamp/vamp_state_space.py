from ompl import base as ob
import vamp

class VampMotionValidator(ob.MotionValidator):
    """Motion validator using VAMP collision checking"""
    
    def __init__(self, 
                si : ob.SpaceInformation, 
                env: vamp.Environment, 
                robot : vamp.robot):
        super().__init__(si)
        self.env = env
        self.robot = robot
        self.dimension = robot.dimension()

    def checkMotion(self, s1: ob.RealVectorStateType, s2: ob.RealVectorStateType) -> bool:
        config1 = s1[0:self.dimension]
        config2 = s2[0:self.dimension]
        return self.robot.validate_motion(config1, config2, self.env)

class VampStateValidityChecker(ob.StateValidityChecker):
    def __init__(self,
                si: ob.SpaceInformation,
                env: vamp.Environment,
                robot: vamp.robot):
        super().__init__(si)
        self.env = env
        self.robot = robot
        self.dimension = robot.dimension()

    def isValid(self, s: ob.RealVectorStateType) -> bool:
        config = s[0:self.dimension]
        return self.robot.validate(config, self.env)

def isStateValid(s: ob.RealVectorStateType, env: vamp.Environment, robot: vamp.robot, dimension: int) -> bool:
    """Check if a state is valid (collision-free)"""
    config = s[0:dimension]
    return robot.validate(config, env)

class VampStateSpace(ob.RealVectorStateSpace):
    """ State Space class using VAMP's robot methods """
    
    def __init__(self, robot: vamp.robot):
        super().__init__(robot.dimension())
        self.robot = robot
        self.dimension = robot.dimension()
        bounds = ob.RealVectorBounds(self.dimension)
        upper_bounds = robot.upper_bounds()
        lower_bounds = robot.lower_bounds()

        for i in range(self.dimension):
            bounds.setLow(i, lower_bounds[i])
            bounds.setHigh(i, upper_bounds[i])
        self.setBounds(bounds)