#include <boost/test/unit_test.hpp>
#include <templ/Mission.hpp>
#include <templ/io/MissionReader.hpp>
#include <templ/solvers/csp/TransportNetwork.hpp>
#include <moreorg/vocabularies/OM.hpp>

#include "../test_utils.hpp"

using namespace templ;
using namespace moreorg;
namespace pa = templ::solvers::temporal::point_algebra;

struct TransportNetworkSetup
{
    TransportNetworkSetup()
    {
        using namespace ::templ::symbols;
        for(size_t i = 0; i < 10; ++i)
        {
            t.push_back(pa::QualitativeTimePoint::getInstance("t" +
                        std::to_string(i)));

            l.push_back(make_shared<constants::Location>("loc" + std::to_string(i), base::Point(i,i,0)));
        }
    }


    symbols::constants::Location::PtrList l;
    pa::TimePoint::PtrList t;

};

BOOST_AUTO_TEST_SUITE(csp_transport_network)

BOOST_FIXTURE_TEST_CASE(mission_0, TransportNetworkSetup)
{
    // mission outline -- qualitative planning
    // [location_image_provider, mission_point1]@[t0,t1]
    // [location_image_provider, mission_point2]@[t2,t3]
    //
    // temporalConstraint t1 < t2
    //
    // --> translate into systems and update timings accordingly
    // --> mission should contain sychronization points

    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");

    Mission baseMission(om);


    baseMission.addResourceLocationCardinalityConstraint(l[0], t[0], t[1], location_image_provider);
    baseMission.addResourceLocationCardinalityConstraint(l[1], t[2], t[3], location_image_provider);

    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[1],t[2], pa::QualitativeTimePointConstraint::Less));
    baseMission.prepareTimeIntervals();

    {
        pa::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
        solvers::temporal::Interval i0(t[0],t[1], comparator);
        solvers::temporal::Interval i1(t[2],t[3], comparator);

        BOOST_REQUIRE_MESSAGE(i0.before(i1), "Interval before (in time) than other");
    }

    using namespace solvers;
    {
        Mission::Ptr mission = make_shared<Mission>(baseMission);
        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission->setAvailableResources(modelPool);
        BOOST_REQUIRE_MESSAGE(mission->getOrganizationModel(), "Mission has organization model set");

        csp::TransportNetwork::SolutionList solutions = solvers::csp::TransportNetwork::solve(mission, 1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solution found with " << modelPool.toString());
    }

    {
        Mission::Ptr mission = make_shared<Mission>(baseMission);
        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        mission->setAvailableResources(modelPool);

        csp::TransportNetwork::SolutionList solutions = solvers::csp::TransportNetwork::solve(mission, 1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solution found with " << modelPool.toString());
    }

    {
        Mission::Ptr mission = make_shared<Mission>(baseMission);
        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        mission->setAvailableResources(modelPool);

        csp::TransportNetwork::SolutionList solutions = solvers::csp::TransportNetwork::solve(mission, 1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solution found with " << modelPool.toString());
    }

    {
        Mission::Ptr mission = make_shared<Mission>(baseMission);
        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
        mission->setAvailableResources(modelPool);

        csp::TransportNetwork::SolutionList solutions = solvers::csp::TransportNetwork::solve(mission, 1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solution found with " << modelPool.toString());
    }
}

BOOST_FIXTURE_TEST_CASE(mission_1, TransportNetworkSetup)
{
    // mission outline -- qualitative planning
    // [location_image_provider, mission_point1]@[t0,t1]
    // [location_image_provider, mission_point2]@[t2,t3]
    //
    // temporalConstraint t1 < t2
    //
    // --> translate into systems and update timings accordingly
    // --> mission should contain sychronization points

    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr ts = point_algebra::QualitativeTimePoint::getInstance("ts");
    point_algebra::TimePoint::Ptr te = point_algebra::QualitativeTimePoint::getInstance("te");

    Mission baseMission(om);
    baseMission.addResourceLocationCardinalityConstraint(l[0], t[0], t[1], location_image_provider);
    baseMission.addResourceLocationCardinalityConstraint(l[1], t[2], t[3], location_image_provider);

    // Overlapping intervals at two locations
    // te t0 t2 t1 t3
    // t1 > t2, t0 < t3
    //
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(te,ts, point_algebra::QualitativeTimePointConstraint::Greater));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[0],te, point_algebra::QualitativeTimePointConstraint::Greater));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[1],t[2], point_algebra::QualitativeTimePointConstraint::Greater));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[0],t[3], point_algebra::QualitativeTimePointConstraint::Less));
    baseMission.prepareTimeIntervals();

    {
        solvers::temporal::point_algebra::TimePointComparator comparator(baseMission.getTemporalConstraintNetwork());
        Interval i0(t[0],t[1], comparator);
        Interval i1(t[2],t[3], comparator);

        BOOST_REQUIRE_MESSAGE(i0.distinctFrom(i1), "Interval are distinct (in time) from other");
        BOOST_REQUIRE_MESSAGE(i0.overlaps(i1), "Interval overlaps (in time) the other");
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission->setAvailableResources(modelPool);

        // The number of available resources can never fulfill the overlapping
        // requirement, hence the planner should throw here
        BOOST_REQUIRE_THROW(solvers::csp::TransportNetwork::solve(mission,1), std::runtime_error);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        mission->setAvailableResources(modelPool);
        // Set starting position of resources
        mission->addResourceLocationCardinalityConstraint(l[0], ts, te, vocabulary::OM::resolve("Sherpa"), 2);
        mission->prepareTimeIntervals();

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 10;
        mission->setAvailableResources(modelPool);
        // Set starting position of resources
        mission->addResourceLocationCardinalityConstraint(l[0], ts, te, vocabulary::OM::resolve("Sherpa"), 10);
        mission->prepareTimeIntervals();

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 1;
        mission->setAvailableResources(modelPool);
        // Set starting position of resources
        mission->addResourceLocationCardinalityConstraint(l[0], ts, te, vocabulary::OM::resolve("Sherpa"), 1);
        mission->addResourceLocationCardinalityConstraint(l[0], ts, te, vocabulary::OM::resolve("CREX"), 1);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));

        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 2;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 3;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 10;
        mission->setAvailableResources(modelPool);

        mission->addResourceLocationCardinalityConstraint(l[0], ts, te, vocabulary::OM::resolve("Sherpa"), 1);
        mission->addResourceLocationCardinalityConstraint(l[0], ts, te, vocabulary::OM::resolve("CREX"), 1);

        owlapi::model::IRI emi_power_provider = vocabulary::OM::resolve("EmiPowerProvider");
        mission->addResourceLocationCardinalityConstraint(l[1], t[2], t[3], emi_power_provider);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,1);
        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }
}

BOOST_FIXTURE_TEST_CASE(mission_2, TransportNetworkSetup)
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

    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("ImageProvider");

    Mission baseMission(om);

    baseMission.addResourceLocationCardinalityConstraint(l[0], t[0], t[1], location_image_provider);
    //baseMission.addResourceLocationCardinalityConstraint(l[0], t[0], t[1], vocabulary::OM::resolve("Payload") );
    baseMission.addResourceLocationCardinalityConstraint(l[1], t[6], t[7], location_image_provider);
    //baseMission.addResourceLocationCardinalityConstraint(l[1], t[6], t[7], vocabulary::OM::resolve("Payload") );

    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[0],t[1], pa::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[1],t[2], pa::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[2],t[3], pa::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[3],t[4], pa::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[4],t[5], pa::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[5],t[6], pa::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t[6],t[7], pa::QualitativeTimePointConstraint::Less));

    baseMission.prepareTimeIntervals();

    using namespace solvers;
    {
        Mission::Ptr mission = make_shared<Mission>(baseMission);
        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        //modelPool[ vocabulary::OM::resolve("CREX") ] = 1;
        //modelPool[ vocabulary::OM::resolve("Payload") ] = 1;
        mission->setAvailableResources(modelPool);
        BOOST_REQUIRE_MESSAGE(mission->getOrganizationModel(), "Mission has organization model set");

        std::vector<solvers::csp::TransportNetwork::Solution> solution = solvers::csp::TransportNetwork::solve(mission,1);
        BOOST_TEST_MESSAGE("solution: " << solution[0].toString());

    }
}

BOOST_AUTO_TEST_CASE(mission_from_file)
{
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(owlapi::model::IRI("http://www.rock-robotics.org/2015/12/projects/TransTerrA"));
    std::string missionFilename = getRootDir() + "test/data/scenarios/should_succeed/0.xml";
    Mission baseMission = templ::io::MissionReader::fromFile(missionFilename, om);
    baseMission.prepareTimeIntervals();

    using namespace solvers;
    {
        Mission::Ptr mission = make_shared<Mission>(baseMission);
        solvers::csp::TransportNetwork::solve(mission,1);
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

    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
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

    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t1,t2, point_algebra::QualitativeTimePointConstraint::Less));
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

        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("CREX") ] = 2;
        modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
        mission->setAvailableResources(modelPool);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,1);
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

    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
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
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t2,t1, point_algebra::QualitativeTimePointConstraint::Greater));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t4,t3, point_algebra::QualitativeTimePointConstraint::Less));
    baseMission.prepareTimeIntervals();

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));
        moreorg::ModelPool modelPool;
        modelPool[ vocabulary::OM::resolve("Payload") ] = 2;
        mission->setAvailableResources(modelPool);

        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission,1);
        BOOST_REQUIRE_MESSAGE(solutions.size() < 10, "Number of solutions found (using symmetry breaking): # of found solutions less than 10 (since symmetry breaking "
                " is not a 'complete' algorithm) : actual number of solutions: " << solutions);
    }
}

BOOST_AUTO_TEST_CASE(mission_3)
// systems: Sherpa, Paylad
// timepoints: t0 --> t5
// locations: loc0-loc2
{
    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
    owlapi::model::IRI sherpa = vocabulary::OM::resolve("Sherpa");
    owlapi::model::IRI payload = vocabulary::OM::resolve("Payload");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");
    point_algebra::TimePoint::Ptr t4 = point_algebra::QualitativeTimePoint::getInstance("t4");
    point_algebra::TimePoint::Ptr t5 = point_algebra::QualitativeTimePoint::getInstance("t5");

    Mission baseMission(om);

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0( new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1( new constants::Location("loc1", base::Point(10,10,0)));
    constants::Location::Ptr loc2( new constants::Location("loc2", base::Point(10,20,0)));

    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, sherpa);
    baseMission.addResourceLocationCardinalityConstraint(loc0, t0, t1, payload);

    baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, payload);
    //baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, sherpa);
    baseMission.addResourceLocationCardinalityConstraint(loc2, t4, t5, sherpa);

    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t0,t1, point_algebra::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t1,t2, point_algebra::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t2,t3, point_algebra::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t3,t4, point_algebra::QualitativeTimePointConstraint::Less));
    baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(t4,t5, point_algebra::QualitativeTimePointConstraint::Less));

    baseMission.prepareTimeIntervals();

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));
        moreorg::ModelPool modelPool;
        modelPool[ sherpa ] = 1;
        modelPool[ payload ] = 1;
        mission->setAvailableResources(modelPool);

        BOOST_REQUIRE_MESSAGE(mission->getOrganizationModel(), "Mission has organization model set");
        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission, 1);

        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }
}

BOOST_AUTO_TEST_CASE(mission_4)
// systems: Sherpa, Paylad
// timepoints: t0 --> t5
// locations: loc0-loc2
{
    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
    owlapi::model::IRI sherpa = vocabulary::OM::resolve("Sherpa");
    owlapi::model::IRI payload = vocabulary::OM::resolve("Payload");

    Mission baseMission(om);

    using namespace solvers::temporal;
    point_algebra::TimePoint::PtrList timepoints;
    for(int t = 0; t < 6; ++t)
    {
        std::stringstream ss;
        ss << "tp" << t;

        point_algebra::TimePoint::Ptr tp = point_algebra::QualitativeTimePoint::getInstance( ss.str() );
        timepoints.push_back(tp);
        if(t > 0)
        {
            baseMission.addConstraint(make_shared<pa::QualitativeTimePointConstraint>(timepoints[t-1],tp, point_algebra::QualitativeTimePointConstraint::Less));
        }
    }

    using namespace ::templ::symbols;
    constants::Location::PtrList locations;
    for(int l = 0; l < 5; ++l)
    {
        std::stringstream ss;
        ss << "loc" << l;
        constants::Location::Ptr loc( new constants::Location(ss.str(), base::Point(l,0,0)));
        locations.push_back(loc);
    }

    baseMission.addResourceLocationCardinalityConstraint(locations[0], timepoints[0], timepoints[1], sherpa, 5);
    baseMission.addResourceLocationCardinalityConstraint(locations[0], timepoints[0], timepoints[1], payload, 10);

    for(int l = 2; l < 3; ++l)
    {
        baseMission.addResourceLocationCardinalityConstraint(locations[l], timepoints[4], timepoints[5], payload, 3);
    }


    //baseMission.addResourceLocationCardinalityConstraint(loc1, t2, t3, sherpa);
    for(int l = 1; l < 5; ++l)
    {
        baseMission.addResourceLocationCardinalityConstraint(locations[l], timepoints[2], timepoints[3], sherpa);
    }

    baseMission.prepareTimeIntervals();

    using namespace solvers;
    {
        Mission::Ptr mission(new Mission(baseMission));
        moreorg::ModelPool modelPool;
        modelPool[ sherpa ] = 5;
        modelPool[ payload ] = 10;
        mission->setAvailableResources(modelPool);

        BOOST_REQUIRE_MESSAGE(mission->getOrganizationModel(), "Mission has organization model set");
        std::vector<solvers::csp::TransportNetwork::Solution> solutions = solvers::csp::TransportNetwork::solve(mission, 1);

        BOOST_REQUIRE_MESSAGE(!solutions.empty(), "Solutions found " << solutions);
    }
}
BOOST_AUTO_TEST_SUITE_END()
