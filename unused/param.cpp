#include <boost/any.hpp>

ClassForward(Parameter);
class Parameter
{
public:
	virtual const boost::any& get() const = 0;
	virtual void set(const boost::any &value) = 0;
	virtual boost::any increment(void) const = 0;
	virtual boost::any decrement(void) const = 0;
	bool done(void) = 0;
};

class ParameterSet
{
	std::vector<ParameterPtr> params;
	void load();
	void save();
};

#define EXPOSE_DISCRETE_PARAMETER(name, type -- can be infered from vector::value_type, values -- a std::vector<type>)
typedef values::value_type name##type;
class ParameterImpl##name : public Parameter 
{
	public:

	ParameterImpl##name(void) : Parameter()
	{
	}

	virtual const boost::any& get(void) const
	{
		boost::any value = getParamFn();
		return value;
	}

	virtual void set(const boost::any &value)
	{	
		setParamFn(any_cast<type>(value));
	}

	virtual boost::any increment(void) const
	{
		boost::any value = getParamFn() + 1;
		return value;
	}

	virtual boost::any decrement(void) const 
	{
		boost::any value = getParamFn() + 1;
		return value;
	}

	bool done(void)
	{
		return true;
	}

protected:
	boost::function0<type>       getParamFn;
	boost::function1<void, type> setParamFn;
};

registerGlobalParameter(this, ParameterPtr param(new ParameterImpl##name(boost::bind(get##name, this), boost::bind(set##name, this, _1))));
getParameterSet(this);

static std::map<void*, ParameterSet> params;




class SpaceInformation
{
public:
  double getCD() { return cd_; }
  void setCD(double cd) {cd_ = cd; }

  EXPOSE_PARAMETER(CD, double,

  
  const ParameterSet& getParameters(void) const;
private:
  double cd_;
};

class Planner
{
public:
	double getRange() { return r_; }
	void setRange(double r)  { r_ = r; }
private:
	double r_;
};


/*

class AutoTune
{
public:
	AutoTune(SimpleSetup &setup, ParameterSet &param, unsigned int N = 100)
	{}
	
	
};

class AutoTuneSingle
{
public:
  AutoTuneSingle(SimpleSetup &setup, Parameter &param, unsigned int N = 100)
  {

  }

  virtual double evaluateParam(void)
  {
	double sum = 0.0;
	for (int i = 0 ; i < N ; ++i)
	{
		setup_.clear();
		setup_.solve();
		sum += setup_.getLastPlanningTime();
	}
	return sum / N;
  }

  void tune(void)
  {
	tune(setup_.getPlanner());
  }

  void tune(base::PlannerPtr &planner)
  {
	if (!planner)
		scream;

	setup_.setPlanner(planner);
	setup_.setup();

	double t0 = evaluateParam();
	unsigned int steps = 0;
	do 
	{
		steps++;
		T value = param.get();
		param.set(param.increment());
		double t = evaluateParam();
		if (t >= t0)
		{
			param.set(value);	
			break;
		}
	} while (true);
	if (steps < 2)
	do 
	{
		T value = param.get();
		param.set(param.decrement());
		double t = evaluateParam();
		if (t >= t0)
		{
			param.set(value);	
			break;
		}
	} while (true);	
  }
}
*/

int main()
{
return 0;
}
