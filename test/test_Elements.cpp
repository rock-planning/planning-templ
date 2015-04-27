#include <boost/test/unit_test.hpp>
#include <templ/PlannerElement.hpp>

using namespace templ;

BOOST_AUTO_TEST_SUITE(elements)

BOOST_AUTO_TEST_CASE(planner_elements)
{
    PlannerElement::Ptr plannerElement0(new PlannerElement("name","type",PlannerElement::STATE_VARIABLE));
    PlannerElement::Ptr plannerElement1(new PlannerElement("name","type",PlannerElement::STATE_VARIABLE));

    PlannerElement::Ptr plannerElement2(new PlannerElement("name2","type2",PlannerElement::STATE_VARIABLE));

    BOOST_REQUIRE_MESSAGE(plannerElement0->equals(plannerElement1), "Elements are equal");
    BOOST_REQUIRE_MESSAGE(!plannerElement0->equals(plannerElement2), "Elements are not equal");
}

BOOST_AUTO_TEST_SUITE_END()



