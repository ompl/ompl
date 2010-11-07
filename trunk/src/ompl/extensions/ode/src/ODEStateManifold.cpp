#include "ompl/extensions/ode/ODEStateManifold.h"
#include <limits>
#include <queue>

const int ompl::control::ODEStateManifold::ODEStateManifold::STATE_COLLISION_TRUE     = 1;
const int ompl::control::ODEStateManifold::ODEStateManifold::STATE_COLLISION_FALSE    = -1;
const int ompl::control::ODEStateManifold::ODEStateManifold::STATE_COLLISION_UNKNOWN  = 0;

ompl::control::ODEStateManifold::ODEStateManifold(const ODEEnvironment &env) : base::CompoundStateManifold(), env_(env)
{
    for (unsigned int i = 0 ; i < env_.stateBodies_.size() ; ++i)
    {
	addSubManifold(base::StateManifoldPtr(new base::RealVectorStateManifold(3)), 1.0); // position
	addSubManifold(base::StateManifoldPtr(new base::RealVectorStateManifold(3)), 1.0); // linear velocity
	addSubManifold(base::StateManifoldPtr(new base::RealVectorStateManifold(3)), 1.0); // angular velocity
	addSubManifold(base::StateManifoldPtr(new base::SO3StateManifold()), 1.0);         // orientation
    }
    lock();
    setDefaultBounds();
}

void ompl::control::ODEStateManifold::setDefaultBounds(void)
{   
    // limit all velocities to 1 m/s, 1 rad/s, respectively
    base::RealVectorBounds bounds1(3);
    bounds1.setLow(-1);
    bounds1.setHigh(1);
    setLinearVelocityBounds(bounds1);
    setAngularVelocityBounds(bounds1);
    
    // find the bounding box that contains all geoms included in the collision spaces
    double mX, mY, mZ, MX, MY, MZ;
    mX = mY = mZ = std::numeric_limits<double>::infinity();
    MX = MY = MZ = -std::numeric_limits<double>::infinity();
    bool found = false;
    
    std::queue<dSpaceID> spaces;
    for (unsigned int i = 0 ; i < env_.collisionSpaces_.size() ; ++i)
	spaces.push(env_.collisionSpaces_[i]);
    
    while (!spaces.empty())
    {
	dSpaceID space = spaces.front();
	spaces.pop();
	
	int n = dSpaceGetNumGeoms(space);
	for (int j = 0 ; j < n ; ++j)
	{
	    dGeomID geom = dSpaceGetGeom(space, j);
	    if (dGeomIsSpace(geom))
		spaces.push((dSpaceID)geom);
	    else
	    {	
		found = true;
		dReal aabb[6];
		dGeomGetAABB(geom, aabb);
		if (aabb[0] < mX) mX = aabb[0];
		if (aabb[1] > MX) MX = aabb[1];
		if (aabb[2] < mY) mY = aabb[2];
		if (aabb[3] > MY) MY = aabb[3];
		if (aabb[4] < mZ) mZ = aabb[4];
		if (aabb[5] > MZ) MZ = aabb[5];
	    }
	}
    }
    
    if (found)
    {
	double dx = MX - mX;
	double dy = MY - mY;
	double dz = MZ - mZ;
	double dM = std::max(dx, std::max(dy, dz));
	
	// add 10% in each dimension + 1% of the max dimension
	dx = dx / 10.0 + dM / 100.0;
	dy = dy / 10.0 + dM / 100.0;
	dz = dz / 10.0 + dM / 100.0;
	
	bounds1.low[0] = mX - dx;
	bounds1.high[0] = MX + dx;
	bounds1.low[1] = mY - dy;
	bounds1.high[1] = MY + dy;
	bounds1.low[2] = mZ - dz;
	bounds1.high[2] = MZ + dz;
	
	setVolumeBounds(bounds1);
    }
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
    env_.mutex_.lock();
    writeState(state);
    CallbackParam cp = { &env_, false };
    for (unsigned int i = 0 ; cp.collision == false && i < env_.collisionSpaces_.size() ; ++i)
	dSpaceCollide(env_.collisionSpaces_[i], &cp, &nearCallback);
    env_.mutex_.unlock();
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
    for (unsigned int i = 0 ; i < env_.stateBodies_.size() ; ++i)
	components_[i * 4]->as<base::RealVectorStateManifold>()->setBounds(bounds);
}

void ompl::control::ODEStateManifold::setLinearVelocityBounds(const base::RealVectorBounds &bounds)
{   
    for (unsigned int i = 0 ; i < env_.stateBodies_.size() ; ++i)
	components_[i * 4 + 1]->as<base::RealVectorStateManifold>()->setBounds(bounds);
}

void ompl::control::ODEStateManifold::setAngularVelocityBounds(const base::RealVectorBounds &bounds)
{  
    for (unsigned int i = 0 ; i < env_.stateBodies_.size() ; ++i)
	components_[i * 4 + 2]->as<base::RealVectorStateManifold>()->setBounds(bounds);
}

ompl::base::State* ompl::control::ODEStateManifold::allocState(void) const
{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::control::ODEStateManifold::freeState(base::State *state) const
{
    CompoundStateManifold::freeState(state);
}

void ompl::control::ODEStateManifold::readState(base::State *state) const
{
    StateType *s = state->as<StateType>();
    for (int i = (int)env_.stateBodies_.size() - 1 ; i >= 0 ; --i)
    {
	unsigned int _i4 = i * 4;
	
	const dReal *pos = dBodyGetPosition(env_.stateBodies_[i]);
	const dReal *vel = dBodyGetLinearVel(env_.stateBodies_[i]);
	const dReal *ang = dBodyGetAngularVel(env_.stateBodies_[i]);
	double *s_pos = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	double *s_vel = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	double *s_ang = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	
	for (int j = 0; j < 3; ++j)
	{
	    s_pos[j] = pos[j];
	    s_vel[j] = vel[j];
	    s_ang[j] = ang[j];
	}
	
	const dReal *rot = dBodyGetQuaternion(env_.stateBodies_[i]);
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
    for (int i = (int)env_.stateBodies_.size() - 1 ; i >= 0 ; --i)
    {
	unsigned int _i4 = i * 4;
	
	double *s_pos = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	dBodySetPosition(env_.stateBodies_[i], s_pos[0], s_pos[1], s_pos[2]);
	
	double *s_vel = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	dBodySetLinearVel(env_.stateBodies_[i], s_vel[0], s_vel[1], s_vel[2]);

	double *s_ang = s->as<base::RealVectorStateManifold::StateType>(_i4)->values; ++_i4;
	dBodySetAngularVel(env_.stateBodies_[i],  s_ang[0], s_ang[1], s_ang[2]);
	
    	const base::SO3StateManifold::StateType &s_rot = *s->as<base::SO3StateManifold::StateType>(_i4);
	dQuaternion q;
	q[0] = s_rot.w;
	q[1] = s_rot.x;
	q[2] = s_rot.y;
	q[3] = s_rot.z;
	dBodySetQuaternion(env_.stateBodies_[i], q);
    }
}
