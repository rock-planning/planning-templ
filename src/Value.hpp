#ifndef TEMPL_VALUE_HPP
#define TEMPL_VALUE_HPP

#include <stdexcept>
#include <boost/shared_ptr.hpp>

namespace templ {

/**
 * A Value represents the basic type for all values that can be used throughout
 * planning
 *
 * Note: these values might have to be cast into domains for other (underlying csp
 * solvers that will be used)
 */
class Value
{
public:
    enum Type { UNKNOWN, INT };


    typedef boost::shared_ptr<Value> Ptr;

    /**
     * Retrieve the type for the given value
     */
    Type getType() const { return mType; }

    virtual bool equals(Value::Ptr other) const { throw std::runtime_error("templ::Value::equals not implemented"); }

    /**
     * Default constructor
     */
    Value()
        : mType(UNKNOWN)
    {}

protected:
    Value(Type type)
        : mType(type)
    {}

private:
    /// Datatype of this value
    Type mType;
};

} // end namespace templ
#endif // TEMPL_VALUE_HPP
