#ifndef TEMPL_STATE_VARIABLE_HPP
#define TEMPL_STATE_VARIABLE_HPP

#include <templ/PlannerElement.hpp>

namespace templ {

/**
 * Each StateVariable is a function of time, e.g.
 * rloc(res0) describe the resource location of res0 over time
 */
class StateVariable : public PlannerElement
{
    /// function: e.g. resource location -> mission point 0
    /// resource: e.g. robot -> Sherpa

public:
    StateVariable(const std::string function, const std::string resource)
        : PlannerElement(resource, function, PlannerElement::STATE_VARIABLE)
    {}

    virtual ~StateVariable() {}

    const std::string& getResource() const { return getInstanceName(); }
    const std::string& getFunction() const { return getTypeName(); }
};

} // end namespace templ
#endif // TEMPL_STATE_VARIABLE_HPP
