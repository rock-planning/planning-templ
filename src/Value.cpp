#include "Value.hpp"
#include <boost/assign/list_of.hpp>

namespace templ {

std::map<Value::Type, Value::ValueTypeName> Value::TypeTxt = boost::assign::map_list_of
    (UNKNOWN, "/unknown_t")
    (INT, "/int32_t");

Value::Value()
    : PlannerElement("",TypeTxt[UNKNOWN], PlannerElement::VALUE)
{}

Value::Value(const ValueTypeName& typeName)
    : PlannerElement("", typeName, PlannerElement::VALUE)
{}

} // end namespace templ
