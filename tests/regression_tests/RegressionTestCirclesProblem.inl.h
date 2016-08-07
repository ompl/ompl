
static const unsigned int CIRCLES_ID = 1;

template<>
std::string problemName<CIRCLES_ID>() { return "circles"; }

// Setup for the Circles problem, part of the OMPL test suite.
static std::shared_ptr<geometric::SimpleSetup> setupCirclesProblem(unsigned int query_index) {
    boost::filesystem::path path(TEST_RESOURCES_DIR);

    Circles2D circles;
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circle_queries.txt").string());
    base::SpaceInformationPtr si = geometric::spaceInformation2DCircles(circles);

#if OMPL_VERSION_VALUE < 15000
    // For older versions of OMPL, we are missing the constructor we need, so we hack things.
    geometric::SimpleSetup *raw_ss = new geometric::SimpleSetup(si->getStateSpace());
    const_cast<base::SpaceInformationPtr&>(raw_ss->getSpaceInformation()) = si;
    const_cast<base::ProblemDefinitionPtr&>(raw_ss->getProblemDefinition()).reset(new base::ProblemDefinition(si));
    const_cast<geometric::PathSimplifierPtr&>(raw_ss->getPathSimplifier()).reset(new geometric::PathSimplifier(si));
#else
    geometric::SimpleSetup *raw_ss = new geometric::SimpleSetup(si);
#endif

    std::shared_ptr<geometric::SimpleSetup> ss(raw_ss);

    base::ScopedState<> start(ss->getSpaceInformation());
    base::ScopedState<> goal(ss->getSpaceInformation());
    if (query_index >= circles.getQueryCount())
        return std::shared_ptr<geometric::SimpleSetup>();
    const Circles2D::Query &q = circles.getQuery(query_index);
    start[0] = q.startX_;
    start[1] = q.startY_;
    goal[0] = q.goalX_;
    goal[1] = q.goalY_;
    ss->setStartAndGoalStates(start, goal, 1e-3);
    return ss;
}

// Configure planners for solving this problem.
static base::ProjectionEvaluatorPtr getCirclesProjEvaluator(const base::SpaceInformationPtr &si)
{
    std::vector<double> cdim = {1, 1};
    std::vector<unsigned int> projection = {0, 1};

    return base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection));
}

template<>
void addPlanner<geometric::EST, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto est(std::make_shared<geometric::EST>(si));
    est->setRange(10.0);
    benchmark.addPlanner(est);
}

template<>
void addPlanner<geometric::BiEST, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto est(std::make_shared<geometric::BiEST>(si));
    est->setRange(10.0);
    benchmark.addPlanner(est);
}

template<>
void addPlanner<geometric::ProjEST, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto est(std::make_shared<geometric::ProjEST>(si));
    est->setRange(10.0);
    est->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(est);
}

template<>
void addPlanner<geometric::SBL, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto sbl(std::make_shared<geometric::SBL>(si));
    sbl->setRange(10.0);
    sbl->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(sbl);
}

template<>
void addPlanner<geometric::KPIECE1, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto kpiece(std::make_shared<geometric::KPIECE1>(si));
    kpiece->setRange(10.0);
    kpiece->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(kpiece);
}

template<>
void addPlanner<geometric::BKPIECE1, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto kpiece(std::make_shared<geometric::BKPIECE1>(si));
    kpiece->setRange(10.0);
    kpiece->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(kpiece);
}

template<>
void addPlanner<geometric::LBKPIECE1, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto kpiece(std::make_shared<geometric::LBKPIECE1>(si));
    kpiece->setRange(10.0);
    kpiece->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(kpiece);
}

#if OMPL_VERSION_VALUE >= 13000
template<>
void addPlanner<geometric::PDST, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    auto pdst(std::make_shared<geometric::PDST>(si));
    pdst->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(pdst);
}
#endif
