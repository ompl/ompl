// define this class:
class myStateValidityCheckerClass : public base::StateValidityChecker
{
public:
     myStateValidityCheckerClass(const base::SpaceInformationPtr &si) :
       base::StateValidityChecker(si)
        {
     }

     virtual bool isValid(const base::State *state) const
     {
             return ...;
     }
};
// or this function:
bool myStateValidityCheckerFunction(const base::State *state)
{
     return ...;
}

base::SpaceInformationPtr si(space);
// either this call:
si->setStateValidityChecker(base::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
// or this call:
si->setStateValidityChecker(boost::bind(&myStateValidityCheckerFunction, _1));
si->setStateValidityCheckingResolution(0.03); // 3%
si->setup();
