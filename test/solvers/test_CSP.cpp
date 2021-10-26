#include <boost/test/unit_test.hpp>
#include <templ/Mission.hpp>
#include <templ/solvers/csp/FlawResolution.hpp>
#include <moreorg/vocabularies/OM.hpp>

#include "../test_utils.hpp"

using namespace templ;
using namespace moreorg;
using namespace templ::symbols;

namespace pa = templ::solvers::temporal::point_algebra;

BOOST_AUTO_TEST_SUITE(csp)

//BOOST_AUTO_TEST_CASE(mission_0)
//{
//    // mission outline -- qualitative planning
//    // [location_image_provider, mission_point1]@[t0,t1]
//    // [location_image_provider, mission_point2]@[t2,t3]
//    //
//    // temporalConstraint t1 < t2
//    //
//    // --> translate into systems and update timings accordingly
//    // --> mission should contain sychronization points
//
//    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(
//                getRootDir() + "test/data/om-schema-latest.owl");
//    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");
//
//    using namespace solvers::temporal;
//    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
//    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
//    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
//    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");
//
//    Mission baseMission(om);
//
//    constants::Location::Ptr loc0( new constants::Location("loc0", base::Point(0,0,0)));
//    constants::Location::Ptr loc1( new constants::Location("loc1", base::Point(10,10,0)));
//
//    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
//    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);
//
//    baseMission.addConstraint( make_shared<pa::QualitativeTimePointConstraint>(t1,t2, point_algebra::QualitativeTimePointConstraint::Less));
//    baseMission.prepareTimeIntervals();
//
//    {
//        solvers::temporal::point_algebra::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
//        Interval i0(t0,t1, comparator);
//        Interval i1(t2,t3, comparator);
//
//        BOOST_REQUIRE_MESSAGE(i0.before(i1), "Interval before (in time) than other");
//    }
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
//        mission->setAvailableResources(modelPool);
//        BOOST_REQUIRE_MESSAGE(mission->getOrganizationModel(), "Mission has organization model set");
//
//        solvers::csp::ModelDistribution::solve(mission);
//    }
//
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
//        mission->setAvailableResources(modelPool);
//
//        solvers::csp::ModelDistribution::solve(mission);
//    }
//
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
//        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
//        mission->setAvailableResources(modelPool);
//        solvers::csp::ModelDistribution::solve(mission);
//    }
//
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
//        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
//        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
//        mission->setAvailableResources(modelPool);
//
//        solvers::csp::ModelDistribution::solve(mission);
//    }
//}
//
//BOOST_AUTO_TEST_CASE(mission_1)
//{
//    // mission outline -- qualitative planning
//    // [location_image_provider, mission_point1]@[t0,t1]
//    // [location_image_provider, mission_point2]@[t2,t3]
//    //
//    // temporalConstraint t1 < t2
//    //
//    // --> translate into systems and update timings accordingly
//    // --> mission should contain sychronization points
//
//    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(
//                getRootDir() + "test/data/om-schema-latest.owl");
//    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");
//
//    using namespace solvers::temporal;
//    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
//    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
//    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
//    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");
//
//    using namespace ::templ::symbols;
//    constants::Location::Ptr loc0(new constants::Location("loc0", base::Point(0,0,0)));
//    constants::Location::Ptr loc1(new constants::Location("loc1", base::Point(10,10,0)));
//
//    Mission baseMission(om);
//    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
//    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);
//
//    // Overlapping intervals at two locations
//    baseMission.addConstraint( make_shared<pa::QualitativeTimePointConstraint>(t1,t2, point_algebra::QualitativeTimePointConstraint::Greater));
//    baseMission.addConstraint( make_shared<pa::QualitativeTimePointConstraint>(t0,t3, point_algebra::QualitativeTimePointConstraint::Less));
//    baseMission.prepareTimeIntervals();
//
//    {
//        solvers::temporal::point_algebra::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
//        Interval i0(t0,t1, comparator);
//        Interval i1(t2,t3, comparator);
//
//        BOOST_REQUIRE_MESSAGE(i0.distinctFrom(i1), "Interval are distinct (in time) from other");
//        BOOST_REQUIRE_MESSAGE(i0.overlaps(i1), "Interval overlaps (in time) the other");
//    }
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
//        mission->setAvailableResources(modelPool);
//        BOOST_REQUIRE_THROW(solvers::csp::ModelDistribution::solve(mission), std::runtime_error);
//    }
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 10;
//        mission->setAvailableResources(modelPool);
//        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
//        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
//
//        // Check role distribution after model distribution
//        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0].getModelDistributionSolution());
//        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
//    }
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
//        modelPool[ vocabulary::OM::resolve("CREX") ] = 1;
//        mission->setAvailableResources(modelPool);
//        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
//        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
//
//        // Check role distribution after model distribution
//        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0].getModelDistributionSolution());
//        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
//
//    }
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
//        modelPool[ vocabulary::OM::resolve("CREX") ] = 3;
//        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
//        mission->setAvailableResources(modelPool);
//
//        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
//        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
//
//        // Check role distribution after model distribution
//        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0].getModelDistributionSolution());
//        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
//
//        std::vector<solvers::csp::ModelDistribution::Solution> i_modelSolutions;
//        using namespace solvers::csp;
//        ModelDistribution::SearchState modelSearchState(mission);
//        std::vector<ModelDistribution::SearchState> modelSolutionStates;
//
//        BOOST_TEST_MESSAGE("Checking for iterative production of solutions for model and role distribution");
//        BOOST_REQUIRE_MESSAGE(modelSearchState.getType() == ModelDistribution::SearchState::OPEN, "search state is open");
//        bool stopSearch = false;
//        while(!stopSearch)
//        {
//            ModelDistribution::SearchState nextState = modelSearchState.next();
//            switch(nextState.getType())
//            {
//                case ModelDistribution::SearchState::SUCCESS:
//                    i_modelSolutions.push_back(nextState.getSolution());
//                    modelSolutionStates.push_back(nextState);
//                    break;
//                case ModelDistribution::SearchState::FAILED:
//                    stopSearch = true;
//                    break;
//                case ModelDistribution::SearchState::OPEN:
//                    BOOST_REQUIRE_MESSAGE(false, "Open model search state found though should be either success of failed");
//            }
//        }
//        BOOST_REQUIRE_MESSAGE(solutions.size() == i_modelSolutions.size(), "Number of model solutions of the same size: " << solutions.size() << " vs. " << i_modelSolutions.size());
//        BOOST_TEST_MESSAGE("Iteratively found model distribution solutions: " << i_modelSolutions);
//
//        std::vector<solvers::csp::RoleDistribution::Solution> i_roleSolutions;
//        RoleDistribution::SearchState roleSearchState(modelSolutionStates[0]);
//        stopSearch = false;
//        while(!stopSearch)
//        {
//            RoleDistribution::SearchState nextState = roleSearchState.next();
//            switch(nextState.getType())
//            {
//                case RoleDistribution::SearchState::SUCCESS:
//                    i_roleSolutions.push_back(nextState.getSolution());
//                    break;
//                case RoleDistribution::SearchState::FAILED:
//                    stopSearch = true;
//                    break;
//                case RoleDistribution::SearchState::OPEN:
//                    BOOST_REQUIRE_MESSAGE(false, "Open role search state found, though should be either success or failed");
//            }
//        }
//        BOOST_REQUIRE_MESSAGE(roleSolutions.size() == i_roleSolutions.size(), "Number of role solutions of the same size: " << roleSolutions.size() << " vs. " << i_roleSolutions.size());
//        BOOST_TEST_MESSAGE("Iteratively found role distribution solutions: " << i_roleSolutions);
//    }
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
//        modelPool[ vocabulary::OM::resolve("CREX") ] = 3;
//        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
//        mission->setAvailableResources(modelPool);
//
//        owlapi::model::IRI emi_power_provider = vocabulary::OM::resolve("EmiPowerProvider");
//        mission->addResourceLocationCardinalityConstraint(loc1, t2, t3, emi_power_provider);
//
//        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
//        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
//
//        // Check role distribution after model distribution
//        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0].getModelDistributionSolution());
//        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
//    }
//}
//
//BOOST_AUTO_TEST_CASE(mission_tt)
//{
//
//    // mission outline -- qualitative planning
//    // [location_image_provider, mission_point1]@[t0,t1]
//    // [location_image_provider, mission_point2]@[t2,t3]
//    //
//    // temporalConstraint t1 < t2
//    //
//    // --> translate into systems and update timings accordingly
//    // --> mission should contain sychronization points
//
//    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(
//                getRootDir() + "test/data/om-schema-latest.owl");
//    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");
//
//    using namespace solvers::temporal;
//    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
//    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
//    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
//    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");
//
//    using namespace ::templ::symbols;
//    constants::Location::Ptr loc0(new constants::Location("loc0", base::Point(0,0,0)));
//    constants::Location::Ptr loc1(new constants::Location("loc1", base::Point(10,10,0)));
//
//    Mission baseMission(om);
//    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
//    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);
//
//    baseMission.addConstraint( make_shared<pa::QualitativeTimePointConstraint>(t1,t2, point_algebra::QualitativeTimePointConstraint::Less));
//    baseMission.prepareTimeIntervals();
//
//    {
//        solvers::temporal::point_algebra::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
//        Interval i0(t0,t1, comparator);
//        Interval i1(t2,t3, comparator);
//
//        BOOST_REQUIRE_MESSAGE(i0.before(i1), "Interval before (in time) than other");
//    }
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
//        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
//        mission->setAvailableResources(modelPool);
//
//        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
//        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
//    }
//}
//
//BOOST_AUTO_TEST_CASE(symmetry_breaking)
//{
//    // mission outline -- qualitative planning
//    // [location_image_provider, mission_point1]@[t0,t1]
//    // [location_image_provider, mission_point2]@[t2,t3]
//    //
//    // temporalConstraint t1 < t2
//    //
//    // --> translate into systems and update timings accordingly
//    // --> mission should contain sychronization points
//
//    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(
//                getRootDir() + "test/data/om-schema-latest.owl");
//    owlapi::model::IRI payloadModel = vocabulary::OM::resolve("Payload");
//
//    using namespace solvers::temporal;
//    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
//    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
//    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
//    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");
//    point_algebra::TimePoint::Ptr t4 = point_algebra::QualitativeTimePoint::getInstance("t4");
//    point_algebra::TimePoint::Ptr t5 = point_algebra::QualitativeTimePoint::getInstance("t5");
//
//    using namespace ::templ::symbols;
//    constants::Location::Ptr loc0(new constants::Location("loc0", base::Point(0,0,0)));
//    constants::Location::Ptr loc1(new constants::Location("loc1", base::Point(10,10,0)));
//    constants::Location::Ptr loc2(new constants::Location("loc2", base::Point(10,15,0)));
//
//    Mission baseMission(om);
//    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, payloadModel);
//    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, payloadModel);
//    baseMission.addResourceLocationCardinalityConstraint(loc2, t4, t5, payloadModel);
//
//    // Overlapping intervals at two locations
//    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t2,t1, pa::QualitativeTimePointConstraint::Greater) );
//    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t4,t3, pa::QualitativeTimePointConstraint::Less) );
//    baseMission.prepareTimeIntervals();
//
//    using namespace solvers;
//    {
//        Mission::Ptr mission(new Mission(baseMission));
//        moreorg::ModelPool modelPool;
//        modelPool[ vocabulary::OM::resolve("Payload") ] = 100;
//        mission->setAvailableResources(modelPool);
//
//        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
//        BOOST_REQUIRE_MESSAGE(solutions.size() < 10, "Number of solutions found (using symmetry breaking): # of found solutions less than 10 (since symmetry breaking "
//                " is not a 'complete' algorithm) : actual number of solutions: " << solutions);
//
//        // Check role distribution after model distribution
//        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0].getModelDistributionSolution());
//        BOOST_REQUIRE_MESSAGE(roleSolutions.size() < 10, "Number of solution founds (using symmetry breaking): # of found solutions less than 10 (since symmetry "
//            "breaking is not complete: action number of solutions: " << roleSolutions.size());
//    }
//}

BOOST_AUTO_TEST_CASE(flaw_resolution)
{
    using namespace templ::solvers::csp;
    using namespace templ::solvers;
    using namespace graph_analysis::algorithms;

    std::vector<transshipment::Flaw> flaws;
    graph_analysis::algorithms::ConstraintViolation violation0(MultiCommodityVertex::Ptr(),
            0,0,1,1,ConstraintViolation::MinFlow);

    constants::Location::Ptr loc0 = make_shared<constants::Location>("loc0", base::Point(0,0,0));
    pa::TimePoint::Ptr t0 = pa::QualitativeTimePoint::getInstance("t0");
    SpaceTime::Point spacetime(loc0,t0);

    transshipment::Flaw flaw0(violation0, Role(),spacetime);
    flaws.push_back(flaw0);

    graph_analysis::algorithms::ConstraintViolation violation1(MultiCommodityVertex::Ptr(),
            0,0,1,1,ConstraintViolation::TransFlow);
    transshipment::Flaw flaw1(violation1, Role(), spacetime);
    flaws.push_back(flaw1);

    {
        FlawResolution flawResolution;
        flawResolution.prepare(flaws);
        BOOST_REQUIRE_MESSAGE(flawResolution.remainingDraws().size() == 1, "Flaw resolution options should be 1 but was " <<
                flawResolution.remainingDraws().size());

        while(flawResolution.next(false))
        {
            std::stringstream ss;
            BOOST_TEST_MESSAGE("Draw:");
            FlawResolution::ResolutionOptions draw = flawResolution.current();
            for(FlawResolution::ResolutionOption d : draw)
            {
                BOOST_TEST_MESSAGE("    " << ConstraintViolation::TypeTxt[ d.first.getViolation().getType() ] << " - alternative: " << d.second);
            }
        }
    }

    {

        FlawResolution flawResolution;
        flawResolution.prepare(flaws);
        BOOST_REQUIRE_MESSAGE(flawResolution.remainingDraws().size() == 1, "Flaw resolution options should be 1 but was " <<
                flawResolution.remainingDraws().size());

        while(flawResolution.next())
        {

            std::stringstream ss;
            BOOST_TEST_MESSAGE("Draw:");
            FlawResolution::ResolutionOptions draw = flawResolution.current();
            for(FlawResolution::ResolutionOption d : draw)
            {
                BOOST_TEST_MESSAGE("    " << ConstraintViolation::TypeTxt[ d.first.getViolation().getType() ] << " - alternative: " << d.second);
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
