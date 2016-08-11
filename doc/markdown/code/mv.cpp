// define this class:
class myMotionValidator : public base::MotionValidator
{
public:
    // implement checkMotion()
};

base::SpaceInformationPtr si(space);
si->setMotionValidator(std::make_shared<myMotionValidator>(si));
si->setup();
