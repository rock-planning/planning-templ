#ifndef TEMPL_CAPACITY_LINK_HPP
#define TEMPL_CAPACITY_LINK_HPP

#include <graph_analysis/Edge.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/Role.hpp>

namespace templ {

class CapacityLink : public graph_analysis::Edge
{
public:
    typedef shared_ptr<CapacityLink> Ptr;

    CapacityLink(const Role& role, uint32_t capacity);

    virtual ~CapacityLink();

    virtual std::string getClassName() const{ return "CapacityLink"; }

    virtual std::string toString(uint32_t indent = 0) const;

    void addUser(const Role& role, uint32_t capacity = 1);

    uint32_t getRemainingCapacity();

    const Role& getProvider() const { return mProvider; }

protected:
    virtual graph_analysis::Edge* clone() { return new CapacityLink(*this); }

private:
    Role mProvider;
    uint32_t mMaxCapacity;
    std::map<Role, uint32_t> mUsedCapacity;
};

} // end namespace templ
#endif // TEMPL_CAPACITY_LINK_HPP
