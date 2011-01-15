   // define this class:
   class myMotionValidator : public base::MotionValidator
   {
   public:
       // implement checkMotion()
   };

   base::SpaceInformationPtr si(manifold);
   si->setMotionValidator(base::MotionValidatorPtr(new myMotionValidator(si)));
   si->setup();
