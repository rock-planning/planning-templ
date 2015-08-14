#include "Constant.hpp"
#include <boost/assign/list_of.hpp>

namespace templ {
namespace symbols {

std::map<Constant::Type, std::string> Constant::TypeTxt = boost::assign::map_list_of
    (UNKNOWN, "UNKNOWN")
    (LOCATION, "LOCATION")
    ;

Constant::Constant(const std::string& name, Constant::Type type)
    : Symbol(name, TypeTxt[type], Symbol::CONSTANT)
    , mConstantType(type)
{}

} // end namespace symbols
} // end namespace templ

