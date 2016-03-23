#include <boost/test/unit_test.hpp>
#include <templ/Mission.hpp>
#include <templ/io/MissionReader.hpp>
#include "test_utils.hpp"

using namespace templ;

BOOST_AUTO_TEST_SUITE(mission)

BOOST_AUTO_TEST_CASE(copy_construct)
{
    std::string rootDir = getRootDir();
    std::string missionFilename = rootDir + "/test/data/scenarios/test-mission-0.xml";
    std::string organizationModelFilename = rootDir + "/test/data/om-schema-v0.13.owl";

    Mission mission = io::MissionReader::fromFile(missionFilename);

    using namespace organization_model;
    OrganizationModel::Ptr organizationModel = OrganizationModel::getInstance(organizationModelFilename);
    mission.setOrganizationModel(organizationModel);

    using namespace templ::solvers::temporal;
    point_algebra::TimePoint::Ptr tp0(new point_algebra::QualitativeTimePoint("tp0"));
    point_algebra::TimePoint::Ptr tp1(new point_algebra::QualitativeTimePoint("tp1"));
    mission.addTemporalConstraint(tp0, tp1, point_algebra::QualitativeTimePointConstraint::Greater);

    Mission missionCopy = mission;
    solvers::temporal::TemporalConstraintNetwork* tcn = mission.getTemporalConstraintNetwork().get();
    solvers::temporal::TemporalConstraintNetwork* tcnCopy = missionCopy.getTemporalConstraintNetwork().get();

    BOOST_REQUIRE_MESSAGE(tcn != tcnCopy, "Mission's TemporalConstraintNetworks are different");
    BOOST_REQUIRE_MESSAGE(tcn->getDistanceGraph().get() != tcnCopy->getDistanceGraph().get(), "Mission's TemporalConstraintNetworks DistanceGraphs are different");

    BOOST_REQUIRE_MESSAGE(tcn->getGraph()->size() == tcnCopy->getGraph()->size(), "Graphs have the same size");
    BOOST_REQUIRE_MESSAGE(tcn->getGraph()->order() == tcnCopy->getGraph()->order(), "Graphs have the same order");


}

BOOST_AUTO_TEST_SUITE_END()