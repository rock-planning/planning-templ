#ifndef TEMPL_SYMBOLS_OBJECT_VARIABLE_HPP
#define TEMPL_SYMBOLS_OBJECT_VARIABLE_HPP

#include <vector>
#include <utility>
#include <templ/Symbol.hpp>

namespace templ {
namespace symbols {

class ObjectVariable : public Symbol
{
public:
    typedef shared_ptr<ObjectVariable> Ptr;

    enum Type { UNKNOWN, LOCATION_CARDINALITY };
    static std::map<Type, std::string> TypeTxt;

    ObjectVariable(const std::string& name, Type type = ObjectVariable::UNKNOWN);
    virtual ~ObjectVariable() {}

    static ObjectVariable::Ptr getInstance(const std::string& name, Type type = ObjectVariable::UNKNOWN);

    Type getObjectVariableType() const { return mObjectVariableType; }

    virtual bool equals(const Symbol::Ptr& symbol) const;

private:
    Type mObjectVariableType;

};

typedef std::vector<ObjectVariable::Ptr> ObjectVariableList;

} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_OBJECT_VARIABLE_HPP
