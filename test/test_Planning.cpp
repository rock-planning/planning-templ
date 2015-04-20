#include <boost/test/unit_test.hpp>
#include <templ/Value.hpp>
#include <templ/values/Int.hpp>
#include <templ/solvers/temporal/Event.hpp>
#include <templ/solvers/temporal/PersistenceCondition.hpp>
#include <templ/solvers/temporal/Timeline.hpp>

using namespace templ;
using namespace templ::solvers::temporal;

namespace pa = templ::solvers::temporal::point_algebra;

BOOST_AUTO_TEST_SUITE(planning)

BOOST_AUTO_TEST_CASE(timeline)
{
    StateVariable stateVariable("robot_location","robot0");
    Timeline timeline(stateVariable);

    ObjectVariable::Ptr objectVariable(new ObjectVariable("l0","Location"));
    point_algebra::TimePoint::Ptr fromT(new point_algebra::QualitativeTimePoint("t0"));
    point_algebra::TimePoint::Ptr toT(new point_algebra::QualitativeTimePoint("t1"));


    PersistenceCondition::Ptr assertion0(new PersistenceCondition(stateVariable, objectVariable, fromT, toT));
    timeline.addTemporalAssertion(assertion0);
//    timeline.addConstraint(constraint0);
//
//    timeline.isConsistent();
}

BOOST_AUTO_TEST_CASE(chronicle)
{
//    Chronicle chronicle;
//    chronicle.addTimeline(timeline);
//    chroncile.isConsistent();
//
//    Chronicle otherChronicle;
//    chroncile.isSupporting(otherChronicle);
}

BOOST_AUTO_TEST_CASE(simple_mission)
{}
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
//    the same value and/or the same time points
//
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
