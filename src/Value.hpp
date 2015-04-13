#ifndef TEMPL_VALUE_HPP
#define TEMPL_VALUE_HPP

#include <boost/shared_ptr.hpp>

namespace templ {

/**
 * Value represents the basic type for all values that can be used throughout
 * planning
 *
 * Note: these values might have to be cast into domains for other (underlying csp
 * solvers that will be used)
 */
class Value
{
public:
    enum Type { UNKNOWN, INT };

    Value(Type type)
        : mType(type)
    {}

    typedef boost::shared_ptr<Value> Ptr;

    /**
     * Retrieve the type for
     */
    Type getType() const { return mType; }

    virtual bool equals(Value::Ptr other) const { throw std::runtime_error("templ::Value::equals not implemented"); }

private:
    Type mType;
};

} // end namespace templ
#endif // TEMPL_VALUE_HPP
