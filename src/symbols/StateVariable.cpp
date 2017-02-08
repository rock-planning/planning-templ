#include "StateVariable.hpp"

namespace templ {
namespace symbols{

StateVariable::StateVariable(const std::string& function, const std::string& resource)
        : Symbol(resource, function, Symbol::STATE_VARIABLE)
{}

} // end namespace symbols
} // end namespace temp
