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
si->setStateValidityChecker(std::make_shared<myStateValidityCheckerClass>(si));
// or this call:
si->setStateValidityChecker(myStateValidityCheckerFunction);
si->setStateValidityCheckingResolution(0.03); // 3%
si->setup();
