#ifndef TEMPL_VALUE_HPP
#define TEMPL_VALUE_HPP

#include <map>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <templ/PlannerElement.hpp>

namespace templ {

/**
 * A Value represents the basic type for all values that can be used throughout
 * planning
 *
 * Note: these values might have to be cast into domains for other (underlying csp
 * solvers that will be used)
 */
class Value : public PlannerElement
{
public:
    enum Type { UNKNOWN, INT };

    typedef std::string ValueTypeName;
    static std::map<Type, ValueTypeName> TypeTxt;

    typedef boost::shared_ptr<Value> Ptr;

    virtual bool equals(PlannerElement::Ptr other) const { throw std::runtime_error("templ::Value::equals not implemented"); }

    const std::string& getValueTypeName() const { return getTypeName(); }

    /**
     * Default constructor
     */
    Value();

    virtual ~Value() {};

protected:
    Value(const ValueTypeName& typeName);
};

} // end namespace templ
#endif // TEMPL_VALUE_HPP
