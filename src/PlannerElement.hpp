#ifndef TEMPL_PLANNER_ELEMENT_HPP
#define TEMPL_PLANNER_ELEMENT_HPP

#include <map>
#include <string>
#include <utility>
#include <boost/shared_ptr.hpp>

namespace templ {

class PlannerElement : public std::pair<std::string, std::string>
{
public:
    typedef boost::shared_ptr<PlannerElement> Ptr;

    enum Type { UNKNOWN, STATE_VARIABLE, OBJECT_VARIABLE, CONSTANT, TEMPORAL_VARIABLE, VALUE };
    static std::map<Type, std::string> TypeTxt;

    PlannerElement(const std::string& name, const std::string& type_name, Type type);

    virtual ~PlannerElement() {}

    const std::string& getInstanceName() const { return first; }
    const std::string& getTypeName() const { return second; }

    Type getType() const { return mType; }

    virtual bool equals(PlannerElement::Ptr other) const { return first == other->first && second == other->second; }
private:
    Type mType;
};

} // end namespace templ
#endif // TEMPL_PLANNER_ELEMENT_HPP
