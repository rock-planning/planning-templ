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
        : Value(TypeTxt[INT])
        , mValue(value)
    {}

    virtual ~Int() {}

    bool equals(PlannerElement::Ptr other) const
    { 
        Int::Ptr value = boost::dynamic_pointer_cast<Int>(other);
        if(!value)
        {
            throw std::invalid_argument("templ::values::Int: other is not an instance of Int: cannot compare");

        }

        return mValue == value->mValue;
    }
};

} // end namespace values
} // end namespace templ
#endif // TEMPL_VALUES_INT_HPP
