#include <boost/test/unit_test.hpp>
#include <templ/Symbol.hpp>
#include <templ/symbols/values/Int.hpp>

using namespace templ;
using namespace templ::symbols;

BOOST_AUTO_TEST_SUITE(symbols)

BOOST_AUTO_TEST_CASE(plain)
{
    Symbol::Ptr symbol0(new Symbol("name","type",Symbol::STATE_VARIABLE));
    Symbol::Ptr symbol1(new Symbol("name","type",Symbol::STATE_VARIABLE));

    Symbol::Ptr symbol2(new Symbol("name2","type2",Symbol::STATE_VARIABLE));

    BOOST_REQUIRE_MESSAGE(symbol0->equals(symbol1), "Symbols are equal");
    BOOST_REQUIRE_MESSAGE(!symbol0->equals(symbol2), "Symbols are not equal");
}

BOOST_AUTO_TEST_CASE(values)
{
    Value::Ptr value10(new templ::symbols::values::Int(10));
    Value::Ptr value5(new templ::symbols::values::Int(5));
    Value::Ptr valueOther5(new templ::symbols::values::Int(5));

    BOOST_REQUIRE_MESSAGE(value10->equals(value10), "Value equals self");
    BOOST_REQUIRE_MESSAGE(value5->equals(valueOther5), "Value equals other");
    BOOST_REQUIRE_MESSAGE(!value5->equals(value10), "Value not equals other");
}

BOOST_AUTO_TEST_SUITE_END()



