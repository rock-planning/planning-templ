#include "Value.hpp"
#include <boost/assign/list_of.hpp>
#include <base-logging/Logging.hpp>

namespace templ {
namespace symbols {

std::map<Value::Type, Value::ValueTypeName> Value::TypeTxt = boost::assign::map_list_of
    (UNKNOWN, "/unknown_t")
    (INT, "/int32_t");

Value::Value(Value::Type type)
    : Symbol("", Value::TypeTxt[type], Symbol::VALUE)
    , mValueType(type)
{}


} // end namespace symbols
} // end namespace templ
