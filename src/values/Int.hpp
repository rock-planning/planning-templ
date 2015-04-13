#ifndef TEMPL_VALUES_INT_HPP
#define TEMPL_VALUES_INT_HPP

#include <templ/Value.hpp>

namespace templ {
namespace values {

class Int : public Value
{
    int32_t mValue;

public:
    typedef boost::shared_ptr<Int> Ptr;

    Int(int32_t value)
        : Value(INT)
        , mValue(value)
    {}

    bool equals(Value::Ptr other) const 
    { 
        if(other->getType() != getType())
        {
            throw std::invalid_argument("Incompatible value types: cannot compare");
        } else {
            Int::Ptr otherInt = boost::dynamic_pointer_cast<Int>(other);
            return mValue == otherInt->mValue;
        }
    }
};

} // end namespace values
} // end namespace templ
#endif // TEMPL_VALUES_INT_HPP
