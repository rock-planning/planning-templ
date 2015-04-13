#ifndef TEMPL_STATE_VARIABLE_HPP
#define TEMPL_STATE_VARIABLE_HPP

namespace templ {

/**
 * Each StateVariable is a function of time, e.g.
 * rloc(res0) describe the resource location of res0 over time
 */
class StateVariable
{
    /// function: e.g. resource location -> mission point 0
    /// resource: e.g. robot -> Sherpa
    std::string mFunction;
    std::string mResource;

public:
    StateVariable(const std::string function, const std::string resource)
        : mFunction(function)
        , mResource(resource)
    {}

    const std::string& getFunction() const { return mFunction; }
    const std::string& getResource() const { return mResource; }
};

} // end namespace templ
#endif // TEMPL_STATE_VARIABLE_HPP
