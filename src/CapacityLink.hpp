#ifndef TEMPL_CAPACITY_LINK_HPP
#define TEMPL_CAPACITY_LINK_HPP

#include <graph_analysis/Edge.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/Role.hpp>
#include <graph_analysis/EdgeRegistration.hpp>
#include <boost/serialization/access.hpp>

namespace templ {

class CapacityLink : public graph_analysis::Edge
{
public:
    typedef shared_ptr<CapacityLink> Ptr;

    CapacityLink();

    CapacityLink(const Role& role, uint32_t capacity);

    virtual ~CapacityLink();

    virtual std::string getClassName() const override { return "CapacityLink"; }

    virtual std::string toString(uint32_t indent = 0) const override;

    void addUser(const Role& role, uint32_t capacity = 1);

    uint32_t getRemainingCapacity();

    const Role& getProvider() const { return mProvider; }

    virtual void registerAttributes(graph_analysis::EdgeTypeManager* eManager) const override;

protected:
    virtual graph_analysis::Edge* getClone() const override { return new CapacityLink(*this); }

    std::string serializeProvider() const;
    void deserializeProvider(const std::string& data);

    std::string serializeUsers() const;
    void deserializeUsers(const std::string& data);

private:
    Role mProvider;
    uint32_t mMaxCapacity;
    std::map<Role, uint32_t> mUsedCapacity;

    static const graph_analysis::EdgeRegistration<CapacityLink> msRegistration;
};

} // end namespace templ
#endif // TEMPL_CAPACITY_LINK_HPP
