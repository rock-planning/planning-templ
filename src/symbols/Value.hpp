#ifndef TEMPL_VALUE_HPP
#define TEMPL_VALUE_HPP

#include <map>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <templ/Symbol.hpp>

namespace templ {
namespace symbols {

/**
 * A Value represents the basic type for all values that can be used throughout
 * planning
 *
 * Note: these values might have to be cast into domains for other (underlying csp
 * solvers that will be used)
 */
class Value : public Symbol
{
public:
    enum Type { UNKNOWN, INT };

    typedef std::string ValueTypeName;
    static std::map<Type, ValueTypeName> TypeTxt;

    typedef boost::shared_ptr<Value> Ptr;

    virtual bool equals(const Symbol::Ptr& other) const { throw std::runtime_error("templ::Value::equals not implemented"); }

    /**
     * Default constructor
     */
    Value(Value::Type type = UNKNOWN);

    virtual ~Value() {};

    Type getValueType() const { return mValueType; }

protected:
    Type mValueType;
};

} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_VALUE_HPP
