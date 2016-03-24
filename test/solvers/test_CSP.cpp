#include <boost/test/unit_test.hpp>
#include <templ/Mission.hpp>
#include <templ/solvers/csp/ModelDistribution.hpp>
#include <templ/solvers/csp/RoleDistribution.hpp>
#include <organization_model/vocabularies/OM.hpp>

#include "../test_utils.hpp"

using namespace templ;
using namespace organization_model;

BOOST_AUTO_TEST_SUITE(csp)

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

    Mission mission(om);

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0( new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1( new constants::Location("loc1", base::Point(10,10,0)));

    mission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    mission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);

    mission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Less);

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(mission.getTemporalConstraintNetwork());
        Interval i0(t0,t1, comparator);
        Interval i1(t2,t3, comparator);

        BOOST_REQUIRE_MESSAGE(i0.before(i1), "Interval before (in time) than other");
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission.setAvailableResources(modelPool);
        BOOST_REQUIRE_MESSAGE(mission.getOrganizationModel(), "Mission has organization model set");
        solvers::csp::ModelDistribution::solve(mission);
    }

    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        mission.setAvailableResources(modelPool);
        solvers::csp::ModelDistribution::solve(mission);
    }

    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        mission.setAvailableResources(modelPool);
        solvers::csp::ModelDistribution::solve(mission);
    }

    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
        mission.setAvailableResources(modelPool);
        solvers::csp::ModelDistribution::solve(mission);
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

    Mission mission(om);
    mission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    mission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);

    // Overlapping intervals at two locations
    mission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Greater);
    mission.addTemporalConstraint(t0,t3, point_algebra::QualitativeTimePointConstraint::Less);

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(mission.getTemporalConstraintNetwork());
        Interval i0(t0,t1, comparator);
        Interval i1(t2,t3, comparator);

        BOOST_REQUIRE_MESSAGE(i0.distinctFrom(i1), "Interval are distinct (in time) from other");
        BOOST_REQUIRE_MESSAGE(i0.overlaps(i1), "Interval overlaps (in time) the other");
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission.setAvailableResources(modelPool);
        BOOST_REQUIRE_THROW(solvers::csp::ModelDistribution::solve(mission), std::runtime_error);
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 10;
        mission.setAvailableResources(modelPool);
        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);

        // Check role distribution after model distribution
        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 1;
        mission.setAvailableResources(modelPool);
        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);

        // Check role distribution after model distribution
        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 3;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
        mission.setAvailableResources(modelPool);
        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);

        // Check role distribution after model distribution
        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 3;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
        mission.setAvailableResources(modelPool);

        owlapi::model::IRI emi_power_provider = vocabulary::OM::resolve("EmiPowerProvider");
        mission.addResourceLocationCardinalityConstraint(loc1, t2, t3, emi_power_provider);
            
        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);

        // Check role distribution after model distribution
        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
        BOOST_REQUIRE_MESSAGE(!roleSolutions.empty(), "Solutions found for role distribution: " << roleSolutions);
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

    Mission mission(om);
    mission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    mission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider); 

    mission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Less);

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(mission.getTemporalConstraintNetwork());
        Interval i0(t0,t1, comparator);
        Interval i1(t2,t3, comparator);

        BOOST_REQUIRE_MESSAGE(i0.before(i1), "Interval before (in time) than other");
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission.setAvailableResources(modelPool);

        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
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

    Mission mission(om);
    mission.addResourceLocationCardinalityConstraint(loc0, t0, t1, payloadModel);
    mission.addResourceLocationCardinalityConstraint(loc1, t2, t3, payloadModel);
    mission.addResourceLocationCardinalityConstraint(loc2, t4, t5, payloadModel);

    // Overlapping intervals at two locations
    mission.addTemporalConstraint(t2,t1, point_algebra::QualitativeTimePointConstraint::Greater);
    mission.addTemporalConstraint(t4,t3, point_algebra::QualitativeTimePointConstraint::Less);

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 100;
        mission.setAvailableResources(modelPool);

        std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
        BOOST_REQUIRE_MESSAGE(solutions.size() == 1, "Exactly one solution found " << solutions);

        // Check role distribution after model distribution
        std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
        BOOST_REQUIRE_MESSAGE(roleSolutions.size(), "Exactly one solution found (using symmetry breaking): # of found solutions " << roleSolutions.size());
    }
}

BOOST_AUTO_TEST_SUITE_END()
