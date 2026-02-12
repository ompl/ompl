from ompl import base as ob
import vamp

class VampMotionValidator(ob.MotionValidator):
    """Motion validator using VAMP collision checking"""
    
    def __init__(self, si, env: vamp.Environment, robot_module, dimension: int):
        super().__init__(si)
        self.env = env
        self.robot_module = robot_module
        self.dimension = dimension

    def checkMotion(self, s1: ob.RealVectorStateType, s2: ob.RealVectorStateType) -> bool:
        config1 = s1[0:self.dimension]
        config2 = s2[0:self.dimension]
        return self.robot_module.validate_motion(config1, config2, self.env)


def isStateValid(s: ob.RealVectorStateType, env: vamp.Environment, robot_module, dimension: int) -> bool:
    """Check if a state is valid (collision-free)"""
    config = s[0:dimension]
    return robot_module.validate(config, env)