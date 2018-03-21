#ifndef TEMPL_CAPACITY_LINK_HPP
#define TEMPL_CAPACITY_LINK_HPP

#include <graph_analysis/Edge.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/Role.hpp>
#include <graph_analysis/EdgeRegistration.hpp>
#include <boost/serialization/access.hpp>

namespace templ {

/**
 * \class CapacityLink
 * \brief Represent a link with an upper bound on the capacity it can transport
 * \details A capacity link allows for a detailled description of the link
 * provider and users including an further inspection of the capacities that are
 * used by these users
 */
class CapacityLink : public graph_analysis::Edge
{
public:
    typedef shared_ptr<CapacityLink> Ptr;

    CapacityLink();

    CapacityLink(const Role& role, uint32_t capacity);

    virtual ~CapacityLink();

    virtual std::string getClassName() const override { return "CapacityLink"; }

    virtual std::string toString(uint32_t indent = 0) const override;

    /**
     * Add a user of this capacity link
     */
    void addConsumer(const Role& role, uint32_t capacity = 1);

    /**
     * Get total capacity
     */
    uint32_t getCapacity() const { return mMaxCapacity; }

    /**
     * Return the consumption level in percent of the max capacity
     */
    double getConsumptionLevel() const;

    /**
     * Get used capacity
     */
    uint32_t getUsedCapacity() const;

    /**
     * Get the remaining capacity of this capacity link
     */
    uint32_t getRemainingCapacity() const;

    /**
     * Get the provider for this capacity link
     */
    const Role::Set& getProviders() const { return mProviders; }

    /**
     * Add a provider to this capacity link
     */
    void addProvider(const Role& role, uint32_t capacity);

    /**
     * Register attributes for serialization of this capacity link
     */
    virtual void registerAttributes(graph_analysis::EdgeTypeManager* eManager) const override;

    static const Role& getLocalTransitionRole() { return msLocationTransitionRole; }

    /**
     * Get all roles that are associate with this capacity edge
     */
    const Role::Set& getAllRoles() const;

protected:
    virtual graph_analysis::Edge* getClone() const override { return new CapacityLink(*this); }

    std::string serializeProviders() const;
    void deserializeProviders(const std::string& data);

    std::string serializeConsumers() const;
    void deserializeConsumers(const std::string& data);

    std::string serializeMaxCapacity() const;
    void deserializeMaxCapacity(const std::string& data);

private:
    Role::Set mProviders;
    mutable Role::Set mAllRoles;
    uint32_t mMaxCapacity;
    std::map<Role, uint32_t> mAvailableCapacities;
    std::map<Role, uint32_t> mUsedCapacity;

    static Role msLocationTransitionRole;

    static const graph_analysis::EdgeRegistration<CapacityLink> msRegistration;
};

} // end namespace templ
#endif // TEMPL_CAPACITY_LINK_HPP
