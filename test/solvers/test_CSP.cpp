#include <boost/test/unit_test.hpp>
#include <templ/Mission.hpp>
#include <templ/solvers/csp/ModelDistribution.hpp>

#include "../test_utils.hpp"

using namespace templ;

BOOST_AUTO_TEST_SUITE(csp)

BOOST_AUTO_TEST_CASE(gecode)
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
                getRootDir() + "test/data/om-schema-v0.8.owl");
    organization_model::Service location_image_provider( owlapi::vocabulary::OM::resolve("ImageProvider"));

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    ObjectVariable::Ptr mp0 = ObjectVariable::getInstance("mp0","Location");
    ObjectVariable::Ptr mp1 = ObjectVariable::getInstance("mp1","Location");

    Mission mission(om);
    mission.addConstraint(location_image_provider,
            mp0, t0, t1);
    mission.addConstraint(location_image_provider,
            mp1, t2, t3);

    mission.addTemporalConstraint(t1,t2, point_algebra::QualitativeTimePointConstraint::Less);

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(mission.getTemporalConstraintNetwork());
        Interval i0(t0,t1, comparator);
        Interval i1(t2,t3, comparator);

        BOOST_REQUIRE_MESSAGE(i0 < i1, "Interval smaller (in time) than other");
        //BOOST_REQUIRE_MESSAGE(i1 < i0, "Interval smaller (in time) than other");
    }

    using namespace solvers;
    {
        organization_model::ModelPool modelPool;
        modelPool[ owlapi::vocabulary::OM::resolve("Sherpa") ] = 1;
        mission.setResources(modelPool);
        solvers::csp::ModelDistribution::solve(mission);
    }

    {
        organization_model::ModelPool modelPool;
        modelPool[ owlapi::vocabulary::OM::resolve("Sherpa") ] = 2;
        mission.setResources(modelPool);
        solvers::csp::ModelDistribution::solve(mission);
    }

    {
        organization_model::ModelPool modelPool;
        modelPool[ owlapi::vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ owlapi::vocabulary::OM::resolve("CREX") ] = 2;
        mission.setResources(modelPool);
        solvers::csp::ModelDistribution::solve(mission);
    }
}

BOOST_AUTO_TEST_SUITE_END()
