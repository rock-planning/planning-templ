#ifndef TEMPL_STATE_VARIABLE_HPP
#define TEMPL_STATE_VARIABLE_HPP

namespace templ {

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
};

} // end namespace templ
#endif // TEMPL_STATE_VARIABLE_HPP
