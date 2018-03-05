#include <boost/test/unit_test.hpp>
#include <templ/constraints/ModelConstraint.hpp>

using namespace templ;
using namespace owlapi::model;

BOOST_AUTO_TEST_SUITE(constraints)

BOOST_AUTO_TEST_CASE(equality)
{
    using namespace templ::constraints;

    ModelConstraint::Ptr constraint0 = make_shared<ModelConstraint>();
    ModelConstraint::Ptr constraint1 = make_shared<ModelConstraint>();
    ModelConstraint::Ptr constraint2 = make_shared<ModelConstraint>();

    constraint0->setModelConstraintType(ModelConstraint::MAX);
    constraint1->setModelConstraintType(ModelConstraint::MIN);
    constraint2->setModelConstraintType(ModelConstraint::MAX);

    BOOST_REQUIRE_MESSAGE( *constraint0 != *constraint1, "Model constraint are not equal");
    BOOST_REQUIRE_MESSAGE( *constraint0 == *constraint2, "Model constraint are equal");

    Constraint::Ptr c0 = constraint0;
    Constraint::Ptr c1 = constraint1;
    Constraint::Ptr c2 = constraint2;

    BOOST_REQUIRE_MESSAGE( *c0 != *c1, "Constraints are not equal");
    BOOST_REQUIRE_MESSAGE( *c0 == *c2, "Constraints are equal");
}
BOOST_AUTO_TEST_SUITE_END()
