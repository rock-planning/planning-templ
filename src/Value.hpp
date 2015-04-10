#ifndef TEMPL_VALUE_HPP
#define TEMPL_VALUE_HPP

#include <boost/shared_ptr.hpp>

namespace templ {

class Value
{
public:
    enum Type { UNKNOWN, INT };

    Value(Type type)
        : mType(type)
    {}

    typedef boost::shared_ptr<Value> Ptr;

    Type getType() const { return mType; }

    virtual bool equals(Value::Ptr other) const { throw std::runtime_error("templ::Value operator== not implemented"); }

private:
    Type mType;
};

} // end namespace templ
#endif // TEMPL_VALUE_HPP
