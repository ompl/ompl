
static const unsigned int CIRCLES_ID = 1;

template<>
std::string problemName<CIRCLES_ID>() { return "circles"; }

// Setup for the Circles problem, part of the OMPL test suite.
static boost::shared_ptr<geometric::SimpleSetup> setupCirclesProblem(unsigned int query_index) {
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

    boost::shared_ptr<geometric::SimpleSetup> ss(raw_ss);

    base::ScopedState<> start(ss->getSpaceInformation());
    base::ScopedState<> goal(ss->getSpaceInformation());
    if (query_index >= circles.getQueryCount())
        return boost::shared_ptr<geometric::SimpleSetup>();
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
    std::vector<double> cdim;
    cdim.push_back(1);
    cdim.push_back(1);

    std::vector<unsigned int> projection;
    projection.push_back(0);
    projection.push_back(1);

    return base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection));
}

template<>
void addPlanner<geometric::EST, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    geometric::EST *est = new geometric::EST(si);
    est->setRange(10.0);
    est->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(base::PlannerPtr(est));
}

template<>
void addPlanner<geometric::SBL, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    geometric::SBL *sbl = new geometric::SBL(si);
    sbl->setRange(10.0);
    sbl->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(base::PlannerPtr(sbl));
}

template<>
void addPlanner<geometric::KPIECE1, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    geometric::KPIECE1 *kpiece = new geometric::KPIECE1(si);
    kpiece->setRange(10.0);
    kpiece->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(base::PlannerPtr(kpiece));
}

template<>
void addPlanner<geometric::BKPIECE1, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    geometric::BKPIECE1 *kpiece = new geometric::BKPIECE1(si);
    kpiece->setRange(10.0);
    kpiece->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(base::PlannerPtr(kpiece));
}

template<>
void addPlanner<geometric::LBKPIECE1, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    geometric::LBKPIECE1 *kpiece = new geometric::LBKPIECE1(si);
    kpiece->setRange(10.0);
    kpiece->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(base::PlannerPtr(kpiece));
}

#if OMPL_VERSION_VALUE >= 13000
template<>
void addPlanner<geometric::PDST, CIRCLES_ID>(Benchmark &benchmark, const base::SpaceInformationPtr &si)
{
    geometric::PDST *pdst = new geometric::PDST(si);
    pdst->setProjectionEvaluator(getCirclesProjEvaluator(si));
    benchmark.addPlanner(base::PlannerPtr(pdst));
}
#endif
