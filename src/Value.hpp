#ifndef TEMPL_VALUE_HPP
#define TEMPL_VALUE_HPP

namespace templ {

class Value
{
public:
    bool operator==(const Value& other) const { return true; }
};

} // end namespace templ
#endif // TEMPL_VALUE_HPP
