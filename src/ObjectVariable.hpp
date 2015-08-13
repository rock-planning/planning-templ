#ifndef TEMPL_OBJECT_VARIABLE_HPP
#define TEMPL_OBJECT_VARIABLE_HPP

#include <vector>
#include <utility>
#include <templ/PlannerElement.hpp>

namespace templ {

class ObjectVariable : public PlannerElement
{
public:
    typedef boost::shared_ptr<ObjectVariable> Ptr;

    enum Type { UNKNOWN, LOCATION, LOCATION_CARDINALITY };
    static std::map<Type, std::string> TypeTxt;

    ObjectVariable(const std::string& name, const std::string& typeName)
        : PlannerElement(name, typeName, PlannerElement::OBJECT_VARIABLE)
    {}

    static ObjectVariable::Ptr getInstance(const std::string& name, const std::string& typeName);

    virtual ~ObjectVariable() {}
};

typedef std::vector<ObjectVariable::Ptr> ObjectVariableList;

} // end namespace templ
#endif // TEMPL_OBJECT_VARIABLE_HPP
