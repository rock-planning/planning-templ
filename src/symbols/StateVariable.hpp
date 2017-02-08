#ifndef TEMPL_SYMBOLS_STATE_VARIABLE_HPP
#define TEMPL_SYMBOLS_STATE_VARIABLE_HPP

#include <templ/Symbol.hpp>

namespace templ {
namespace symbols {

/**
 * Each StateVariable represents a function of time, e.g.
 * rloc(res0) describe the resource location of res0 over time
 */
class StateVariable : public Symbol
{
    /// function: e.g. resource location -> mission point 0
    /// resource: e.g. robot -> Sherpa

public:
    StateVariable(const std::string& function, const std::string& resource);
    virtual ~StateVariable() {}

    const std::string& getResource() const { return getInstanceName(); }
    const std::string& getFunction() const { return getTypeName(); }
};

} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_STATE_VARIABLE_HPP
