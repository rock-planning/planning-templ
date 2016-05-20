#include <boost/test/unit_test.hpp>
#include <templ/Mission.hpp>
#include <templ/solvers/csp/TransportNetwork.hpp>
#include <organization_model/vocabularies/OM.hpp>

#include "../test_utils.hpp"

using namespace templ;
using namespace organization_model;

BOOST_AUTO_TEST_SUITE(csp_transport_network)

BOOST_AUTO_TEST_CASE(mission_0)
{
    // mission outline -- qualitative planning
    // [location_image_provider, mission_point1]@[t0,t1]
    // [location_image_provider, mission_point2]@[t2,t3]
    //
    // temporalConstraint t1 < t2
    //
    // --> translate into systems and update timings accordingly
    // --> mission should contain sychronization points

    organization_model::OrganizationModel::Ptr om = organization_model::OrganizationModel::getInstance(
                getRootDir() + "test/data/om-schema-latest.owl");
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    Mission baseMission(om);

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0( new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1( new constants::Location("loc1", base::Point(10,10,0)));

    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);

    baseMission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.prepareTimeIntervals();

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
        Interval i0(t0,t1, comparator);
        Interval i1(t2,t3, comparator);

        BOOST_REQUIRE_MESSAGE(i0.before(i1), "Interval before (in time) than other");
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission->setAvailableResources(modelPool);
        BOOST_REQUIRE_MESSAGE(mission->getOrganizationModel(), "Mission has organization model set");

        solvers::csp::TransportNetwork::solve(mission);
    }

    {
        Mission::Ptr mission(new Mission(baseMission));
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        mission->setAvailableResources(modelPool);

        solvers::csp::TransportNetwork::solve(mission);
    }

    {
        Mission::Ptr mission(new Mission(baseMission));
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        mission->setAvailableResources(modelPool);
        solvers::csp::TransportNetwork::solve(mission);
    }

    {
        Mission::Ptr mission(new Mission(baseMission));
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
        mission->setAvailableResources(modelPool);

        solvers::csp::TransportNetwork::solve(mission);
    }
}

BOOST_AUTO_TEST_CASE(mission_1)
{
    // mission outline -- qualitative planning
    // [location_image_provider, mission_point1]@[t0,t1]
    // [location_image_provider, mission_point2]@[t2,t3]
    //
    // temporalConstraint t1 < t2
    //
    // --> translate into systems and update timings accordingly
    // --> mission should contain sychronization points

    organization_model::OrganizationModel::Ptr om = organization_model::OrganizationModel::getInstance(
                getRootDir() + "test/data/om-schema-latest.owl");
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0(new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1(new constants::Location("loc1", base::Point(10,10,0)));

    Mission baseMission(om);
    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);

    // Overlapping intervals at two locations
    baseMission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Greater);
    baseMission.addTemporalConstraint(t0,t3, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.prepareTimeIntervals();

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
        Interval i0(t0,t1, comparator);
        Interval i1(t2,t3, comparator);

        BOOST_REQUIRE_MESSAGE(i0.distinctFrom(i1), "Interval are distinct (in time) from other");
        BOOST_REQUIRE_MESSAGE(i0.overlaps(i1), "Interval overlaps (in time) the other");
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission->setAvailableResources(modelPool);
        BOOST_REQUIRE_THROW(solvers::csp::TransportNetwork::solve(mission), std::runtime_error);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 10;
        mission->setAvailableResources(modelPool);
        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 1;
        mission->setAvailableResources(modelPool);
        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 1;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 1;
        mission->setAvailableResources(modelPool);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);

        std::vector<solvers::csp::TransportNetwork::Solution> i_modelSolutions;
        using namespace solvers::csp;
        TransportNetwork::SearchState modelSearchState(mission);
        std::vector<TransportNetwork::SearchState> modelSolutionStates;

        BOOST_TEST_MESSAGE("Checking for iterative production of solutions for model and role distribution");
        BOOST_REQUIRE_MESSAGE(modelSearchState.getType() == TransportNetwork::SearchState::OPEN, "search state is open");
        bool stopSearch = false;
        while(!stopSearch)
        {
            TransportNetwork::SearchState nextState = modelSearchState.next();
            switch(nextState.getType())
            {
                case TransportNetwork::SearchState::SUCCESS:
                    i_modelSolutions.push_back(nextState.getSolution());
                    modelSolutionStates.push_back(nextState);
                    stopSearch = true;
                    break;
                case TransportNetwork::SearchState::FAILED:
                    stopSearch = true;
                    break;
                case TransportNetwork::SearchState::OPEN:
                    BOOST_REQUIRE_MESSAGE(false, "Open model search state found though should be either success of failed");
            }
        }
        BOOST_REQUIRE_MESSAGE(solutions.size() == i_modelSolutions.size(), "Number of model solutions of the same size: " << solutions.size() << " vs. " << i_modelSolutions.size());
        BOOST_TEST_MESSAGE("Iteratively found model distribution solutions: " << i_modelSolutions);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 3;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
        mission->setAvailableResources(modelPool);

        owlapi::model::IRI emi_power_provider = vocabulary::OM::resolve("EmiPowerProvider");
        mission->addResourceLocationCardinalityConstraint(loc1, t2, t3, emi_power_provider);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }
}

BOOST_AUTO_TEST_CASE(mission_2)
{
    // mission outline -- qualitative planning
    // [location_image_provider, mission_point1]@[t0,t1]
    // [location_image_provider, mission_point2]@[t2,t3]
    // [location_image_provider, mission_point1]@[t6,t7]
    //
    // temporalConstraint t1 < t2
    //
    // --> translate into systems and update timings accordingly
    // --> mission should contain sychronization points

    organization_model::OrganizationModel::Ptr om = organization_model::OrganizationModel::getInstance(
                getRootDir() + "test/data/om-schema-latest.owl");
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");
    point_algebra::TimePoint::Ptr t4 = point_algebra::QualitativeTimePoint::getInstance("t4");
    point_algebra::TimePoint::Ptr t5 = point_algebra::QualitativeTimePoint::getInstance("t5");
    point_algebra::TimePoint::Ptr t6 = point_algebra::QualitativeTimePoint::getInstance("t6");
    point_algebra::TimePoint::Ptr t7 = point_algebra::QualitativeTimePoint::getInstance("t7");

    Mission baseMission(om);

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0( new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1( new constants::Location("loc1", base::Point(10,10,0)));

    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    //baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);
    baseMission.addResourceLocationCardinalityConstraint(loc0, t6, t7, location_image_provider);

    baseMission.addTemporalConstraint(t0,t1, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.addTemporalConstraint(t2,t3, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.addTemporalConstraint(t3,t4, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.addTemporalConstraint(t4,t5, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.addTemporalConstraint(t5,t6, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.addTemporalConstraint(t6,t7, point_algebra::QualitativeTimePointConstraint::Less);

    baseMission.prepareTimeIntervals();

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission->setAvailableResources(modelPool);
        BOOST_REQUIRE_MESSAGE(mission->getOrganizationModel(), "Mission has organization model set");

        solvers::csp::TransportNetwork::solve(mission);
    }
}


BOOST_AUTO_TEST_CASE(mission_tt)
{

    // mission outline -- qualitative planning
    // [location_image_provider, mission_point1]@[t0,t1]
    // [location_image_provider, mission_point2]@[t2,t3]
    //
    // temporalConstraint t1 < t2
    //
    // --> translate into systems and update timings accordingly
    // --> mission should contain sychronization points

    organization_model::OrganizationModel::Ptr om = organization_model::OrganizationModel::getInstance(
                getRootDir() + "test/data/om-schema-latest.owl");
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0(new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1(new constants::Location("loc1", base::Point(10,10,0)));

    Mission baseMission(om);
    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);

    baseMission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.prepareTimeIntervals();

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
        Interval i0(t0,t1, comparator);
        Interval i1(t2,t3, comparator);

        BOOST_REQUIRE_MESSAGE(i0.before(i1), "Interval before (in time) than other");
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission->setAvailableResources(modelPool);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }
}

BOOST_AUTO_TEST_CASE(symmetry_breaking)
{
    // mission outline -- qualitative planning
    // [location_image_provider, mission_point1]@[t0,t1]
    // [location_image_provider, mission_point2]@[t2,t3]
    //
    // temporalConstraint t1 < t2
    //
    // --> translate into systems and update timings accordingly
    // --> mission should contain sychronization points

    organization_model::OrganizationModel::Ptr om = organization_model::OrganizationModel::getInstance(
                getRootDir() + "test/data/om-schema-latest.owl");
    owlapi::model::IRI payloadModel = vocabulary::OM::resolve("Payload");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");
    point_algebra::TimePoint::Ptr t4 = point_algebra::QualitativeTimePoint::getInstance("t4");
    point_algebra::TimePoint::Ptr t5 = point_algebra::QualitativeTimePoint::getInstance("t5");

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0(new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1(new constants::Location("loc1", base::Point(10,10,0)));
    constants::Location::Ptr loc2(new constants::Location("loc2", base::Point(10,15,0)));

    Mission baseMission(om);
    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, payloadModel);
    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, payloadModel);
    baseMission.addResourceLocationCardinalityConstraint(loc2, t4, t5, payloadModel);

    // Overlapping intervals at two locations
    baseMission.addTemporalConstraint(t2,t1, point_algebra::QualitativeTimePointConstraint::Greater);
    baseMission.addTemporalConstraint(t4,t3, point_algebra::QualitativeTimePointConstraint::Less);
    baseMission.prepareTimeIntervals();

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 100;
        mission->setAvailableResources(modelPool);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission);
        BOOST_REQUIRE_MESSAGE(solutions.size() < 10, "Number of solutions found (using symmetry breaking): # of found solutions less than 10 (since symmetry breaking "
                " is not a 'complete' algorithm) : actual number of solutions: " << solutions);
    }
}

BOOST_AUTO_TEST_SUITE_END()
