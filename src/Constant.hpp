#ifndef TEMPL_CONSTANT_HPP
#define TEMPL_CONSTANT_HPP

#include <vector>
#include <templ/PlannerElement.hpp>

namespace templ {

class Constant : public PlannerElement
{
public:
    typedef boost::shared_ptr<Constant> Ptr;

    Constant(const std::string& name, const std::string& type)
        : PlannerElement(name, type, PlannerElement::CONSTANT)
    {}

    virtual ~Constant() {}
};

typedef std::vector<Constant> ConstantList;

} // end namespace templ
#endif // TEMPL_CONSTANT_HPP
