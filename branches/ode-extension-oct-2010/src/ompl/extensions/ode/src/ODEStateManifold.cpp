#include "ompl/extensions/ode/ODEStateManifold.h"

const int ompl::control::ODEStateManifold::ODEStateManifold::STATE_COLLISION_TRUE     = 1;
const int ompl::control::ODEStateManifold::ODEStateManifold::STATE_COLLISION_FALSE    = -1;
const int ompl::control::ODEStateManifold::ODEStateManifold::STATE_COLLISION_UNKNOWN  = 0;

ompl::control::ODEStateManifold::ODEStateManifold(const ODEEnvironment &env) : base::CompoundStateManifold(), env_(env)
{
    for (unsigned int i = 0 ; i < env_.stateBodies.size() ; ++i)
    {
	addSubManifold(base::StateManifoldPtr(new base::RealVectorStateManifold(3)), 1.0);
	addSubManifold(base::StateManifoldPtr(new base::RealVectorStateManifold(3)), 1.0);
	addSubManifold(base::StateManifoldPtr(new base::RealVectorStateManifold(3)), 1.0);
	addSubManifold(base::StateManifoldPtr(new base::SO3StateManifold()), 1.0);
    }
    lock();
}

void ompl::control::ODEStateManifold::copyState(base::State *destination, const base::State *source) const
{
    CompoundStateManifold::copyState(destination, source);
    destination->as<StateType>()->collision = source->as<StateType>()->collision;
}

namespace ompl
{

    /// @cond IGNORE
    struct CallbackParam
    {
	const control::ODEEnvironment *env;
	bool                           collision;
    };
    /// @endcond

    static void nearCallback(void *data, dGeomID o1, dGeomID o2)
    {
	// if a collision has not already been detected 
	if (reinterpret_cast<CallbackParam*>(data)->collision == false)
	{
	    dBodyID b1 = dGeomGetBody(o1);
	    dBodyID b2 = dGeomGetBody(o2);
	    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
	    
	    dContact contact[1];  // one contact is sufficient
	    int numc = dCollide(o1, o2, 1, &contact[0].geom, sizeof(dContact));
	    
	    // check if there is really a collision
	    if (numc)
		// check if the collision is allowed
		reinterpret_cast<CallbackParam*>(data)->collision =
		    !reinterpret_cast<CallbackParam*>(data)->env->isValidCollision(o1, o2, contact[0]);
	}
    }
}

void ompl::control::ODEStateManifold::evaluateCollision(const base::State *state) const
{
    if (state->as<StateType>()->collision != STATE_COLLISION_UNKNOWN)
	return;
    env_.mutex.lock();
    writeState(state);
    CallbackParam cp = { &env_, false };
    for (unsigned int i = 0 ; cp.collision == false && i < env_.collisionSpaces.size() ; ++i)
	dSpaceCollide(env_.collisionSpaces[i], &cp, &nearCallback);
    env_.mutex.unlock();
    state->as<StateType>()->collision = cp.collision ? STATE_COLLISION_TRUE : STATE_COLLISION_FALSE;
}

bool ompl::control::ODEStateManifold::satisfiesBoundsExceptRotation(const StateType *state) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	if (i % 4 != 3)
	    if (!components_[i]->satisfiesBounds(state->components[i]))
		return false;
    return true;
}

void ompl::control::ODEStateManifold::setVolumeBounds(const base::RealVectorBounds &bounds)
{
    for (unsigned int i = 0 ; i < env_.stateBodies.size() ; ++i)
	components_[i * 4]->as<base::RealVectorStateManifold>()->setBounds(bounds);
}

void ompl::control::ODEStateManifold::setLinearVelocityBounds(const base::RealVectorBounds &bounds)
{   
    for (unsigned int i = 0 ; i < env_.stateBodies.size() ; ++i)
	components_[i * 4 + 1]->as<base::RealVectorStateManifold>()->setBounds(bounds);
}

void ompl::control::ODEStateManifold::setAngularVelocityBounds(const base::RealVectorBounds &bounds)
{  
    for (unsigned int i = 0 ; i < env_.stateBodies.size() ; ++i)
	components_[i * 4 + 2]->as<base::RealVectorStateManifold>()->setBounds(bounds);
}

ompl::base::State* ompl::control::ODEStateManifold::allocState(void) const
{
    return new StateType();
}

void ompl::control::ODEStateManifold::freeState(base::State *state) const
{
    CompoundStateManifold::freeState(state);
}

void ompl::control::ODEStateManifold::readState(base::State *state) const
{
    StateType *s = state->as<StateType>();
    for (int i = (int)env_.stateBodies.size() - 1 ; i >= 0 ; --i)
    {
	unsigned int _i4 = i * 4;
	
	const dReal *pos = dBodyGetPosition(env_.stateBodies[i]);
	const dReal *vel = dBodyGetLinearVel(env_.stateBodies[i]);
	const dReal *ang = dBodyGetAngularVel(env_.stateBodies[i]);
	double *s_pos = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	double *s_vel = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	double *s_ang = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	
	for (int j = 0; j < 3; ++j)
	{
	    s_pos[j] = pos[j];
	    s_vel[j] = vel[j];
	    s_ang[j] = ang[j];
	}
	
	const dReal *rot = dBodyGetQuaternion(env_.stateBodies[i]);
    	base::SO3StateManifold::StateType &s_rot = *s->as<base::SO3StateManifold::StateType>(_i4);
	
	s_rot.w = rot[0];
	s_rot.x = rot[1];
	s_rot.y = rot[2];
	s_rot.z = rot[3];
    }
}

void ompl::control::ODEStateManifold::writeState(const base::State *state) const
{ 
    const StateType *s = state->as<StateType>();
    for (int i = (int)env_.stateBodies.size() - 1 ; i >= 0 ; --i)
    {
	unsigned int _i4 = i * 4;
	
	double *s_pos = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	dBodySetPosition(env_.stateBodies[i], s_pos[0], s_pos[1], s_pos[2]);
	
	double *s_vel = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	dBodySetLinearVel(env_.stateBodies[i], s_vel[0], s_vel[1], s_vel[2]);

	double *s_ang = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	dBodySetAngularVel(env_.stateBodies[i],  s_ang[3], s_ang[4], s_ang[5]);
	
    	const base::SO3StateManifold::StateType &s_rot = *s->as<base::SO3StateManifold::StateType>(_i4);
	dQuaternion q;
	q[0] = s_rot.w;
	q[1] = s_rot.x;
	q[2] = s_rot.y;
	q[3] = s_rot.z;
	dBodySetQuaternion(env_.stateBodies[i], q);
    }
}
