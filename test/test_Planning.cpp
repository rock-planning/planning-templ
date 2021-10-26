#include <boost/test/unit_test.hpp>
#include <templ/symbols/Value.hpp>
#include <templ/symbols/values/Int.hpp>
#include <templ/solvers/temporal/Event.hpp>
#include <templ/solvers/temporal/PersistenceCondition.hpp>
#include <templ/solvers/temporal/Timeline.hpp>
#include <templ/solvers/temporal/Chronicle.hpp>
#include "test_utils.hpp"

#include <moreorg/vocabularies/OM.hpp>
#include <moreorg/OrganizationModel.hpp>
#include <templ/Mission.hpp>

using namespace templ;
using namespace templ::symbols;
using namespace templ::solvers::temporal;
using namespace moreorg;

namespace pa = templ::solvers::temporal::point_algebra;

BOOST_AUTO_TEST_SUITE(planning)

BOOST_AUTO_TEST_CASE(chronicle)
{
    StateVariable stateVariable("robot_location","robot0");
    Timeline timeline(stateVariable);

    // rloc(robot0)@[t0,t1) : l0
    ObjectVariable::Ptr l0 = ObjectVariable::getInstance("l0", ObjectVariable::LOCATION_CARDINALITY);
    ObjectVariable::Ptr l1 = ObjectVariable::getInstance("l1", ObjectVariable::LOCATION_CARDINALITY);
    ObjectVariable::Ptr l2 = ObjectVariable::getInstance("l2", ObjectVariable::LOCATION_CARDINALITY);

    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    Event::Ptr event0 = Event::getInstance(stateVariable, l0, l1, t0);
    Event::Ptr event1 = Event::getInstance(stateVariable, l1, l2, t1);
    Event::Ptr event2 = Event::getInstance(stateVariable, l2, l2, t2);
    Event::Ptr event3 = Event::getInstance(stateVariable, l1, l2, t2);

    PersistenceCondition::Ptr pc0 = PersistenceCondition::getInstance(stateVariable, l1, t0, t1);
    PersistenceCondition::Ptr pc1 = PersistenceCondition::getInstance(stateVariable, l2, t1, t2);

    timeline.addTemporalAssertion(event0);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with one event consistent");
    timeline.addTemporalAssertion(pc0);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with one event and one persistence condition consistent");
    timeline.addTemporalAssertion(event1);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with two events and one persistence condition consistent");
    timeline.addTemporalAssertion(pc1);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with two events and two persistence conditions consistent");
    timeline.addTemporalAssertion(event2);
    BOOST_REQUIRE_MESSAGE(timeline.isConsistent(), "Timeline with three events and two persistence conditions is consistent");

    Chronicle chronicle;
    chronicle.addTimeline(timeline);
    BOOST_TEST_MESSAGE(chronicle.toString());
    BOOST_REQUIRE_MESSAGE(chronicle.isConsistent(), "Chronicle is consistent");


//    Chronicle otherChronicle;
//    chroncile.isSupporting(otherChronicle);
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

    owlapi::model::IRI organizationModelIRI = "http://www.rock-robotics.org/2015/12/projects/TransTerrA";
    moreorg::OrganizationModel::Ptr om = moreorg::OrganizationModel::getInstance(organizationModelIRI);
    owlapi::model::IRI location_image_provider = vocabulary::OM::resolve("LocationImageProvider");

    using namespace solvers::temporal;
    point_algebra::TimePoint::Ptr t0 = point_algebra::QualitativeTimePoint::getInstance("t0");
    point_algebra::TimePoint::Ptr t1 = point_algebra::QualitativeTimePoint::getInstance("t1");
    point_algebra::TimePoint::Ptr t2 = point_algebra::QualitativeTimePoint::getInstance("t2");
    point_algebra::TimePoint::Ptr t3 = point_algebra::QualitativeTimePoint::getInstance("t3");

    using namespace ::templ::symbols;
    constants::Location::Ptr loc0( new constants::Location("loc0", base::Point(0,0,0)));
    constants::Location::Ptr loc1( new constants::Location("loc1", base::Point(10,10,0)));

    Mission mission(om);
    mission.addResourceLocationCardinalityConstraint(loc0, t0, t1, location_image_provider);
    mission.addResourceLocationCardinalityConstraint(loc1, t2, t3, location_image_provider);

    using namespace solvers;
    Constraint::Ptr t1_t2_constraint =  point_algebra::QualitativeTimePointConstraint::create(t1,t2, point_algebra::QualitativeTimePointConstraint::Less);
    mission.addConstraint(t1_t2_constraint);

    moreorg::ModelPool modelPool;
    modelPool[ vocabulary::OM::resolve("Sherpa") ] = 1;
    mission.setAvailableResources(modelPool);

}

// actions
//  -- transition, i.e. moving from location A->B
//  -- reconfigure: merge/split
// cost
//  -- transition cost [time, energy, space]
//  -- runtime cost for service
//
// add mission outline
// [image_provider, station1]@[200,1000]
// [location_image_provider, mission_point1]@[100,350]
// [location_image_provider, mission_point2]@[500,800]
// [location_image_provider, mission_point3]@[200,600]

// persistence condition: rloc(r1)@[t1,t2):loc1
// event: rloc(r1)@t1:(l1,loc1)
//
// t1 = 100
// t2 = 200
// t3 = 350
// t4 = 500
// t5 = 600
// t6 = 800
//
//  --> constants: location, resource
//  --> object variables: (can be have to constants)
//  --> temporal variable
// chronicle
// ({
//  resource_location( location_image_provider[0] )@t1 : (location0, mission_point1)
//  resource_location( location_image_provider[0] )@[t1,t3] : mission_point1
//  resource_location( location_image_provider[0] )@[t4,t6] : mission_point2
//  resource_location( location_image_provider[0] )@[t2,t5] : mission_point3
// },
// { t1 < t2 < t3 < t4 < t5 < t6 }
// )
//
// timeline -> chronicle for a single state variable
//    -- isConsistent --> iff every pair of assertions is disjoint or refers to
// chronicle ->
//    -- isConsistent iff the timelines for all the state variables in it are
//    consistent
//
//
// --> will lead to a conflict so introduce another symbol for
//     location_image_provider[1] as possible solution
//
//
// 1. add minimum required resources to timepoints
//
//  --> TPS
//     flaw <-- UnsatifiesfiedAxioms
//     select any flaw
//        <-- resolve and refine plan
//
// 2. check (temporal) consistency with respect to available resources
//    -> find minimum assignment required
//      --> fail if mission would require change OR more resources (to fulfill bare minimum requirements)
//
// 3. cast resources into available systems --> (prefer assignment with least
// commitment: --> use gecode for consistency check?!)
//    -> check (temporal) consistency with respect to available systems
// 4. check transitions between timepoints, i.e. mobility/speed of
//         systems/cost of traversal

BOOST_AUTO_TEST_SUITE_END()
