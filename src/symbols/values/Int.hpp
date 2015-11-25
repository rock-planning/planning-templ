#ifndef TEMPL_SYMBOLS_VALUES_INT_HPP
#define TEMPL_SYMBOLS_VALUES_INT_HPP

#include <templ/symbols/Value.hpp>
#include <base/Logging.hpp>

namespace templ {
namespace symbols {
namespace values {

class Int : public Value
{
    int32_t mInt;

public:
    typedef shared_ptr<Int> Ptr;

    Int(int32_t value)
        : Value(INT)
        , mInt(value)
    {}

    virtual ~Int() {}

    bool equals(const Symbol::Ptr& other) const
    { 
        Int::Ptr value = dynamic_pointer_cast<Int>(other);
        if(!value)
        {
            throw std::invalid_argument("templ::values::Int: other is not an instance of Int: cannot compare");
        }

        return mInt == value->mInt;
    }

    std::string toString() const
    {
        std::stringstream ss;
        ss << "Symbol: " << mInt << " (" << Value::TypeTxt[getValueType()] << ")";
        return ss.str();
    }
};

} // end namespace values
} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_VALUES_INT_HPP
