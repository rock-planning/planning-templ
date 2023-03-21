#include "CapacityLink.hpp"
#include <sstream>
#include <graph_analysis/EdgeTypeManager.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <base-logging/Logging.hpp>

namespace templ {

const graph_analysis::EdgeRegistration<CapacityLink> CapacityLink::msRegistration;

Role CapacityLink::msLocationTransitionRole = Role(0,owlapi::model::IRI("http://rock-robotics.org/2014/07/internal-role#local-transition"));

CapacityLink::CapacityLink()
    : graph_analysis::Edge()
    , mProviders()
    , mMaxCapacity(0)
{}

CapacityLink::CapacityLink(const Role& role, uint32_t capacity)
    : graph_analysis::Edge()
    , mMaxCapacity(capacity)
{
    mProviders.insert(role);
}

CapacityLink::~CapacityLink() {}

std::string CapacityLink::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "CapacityLink " << "(max: " << getCapacity() << ")" << std::endl;
    //if(getSourceVertex())
    //{
    //    ss << hspace << "    from: " << getSourceVertex()->toString() << std::endl;
    //} else {
    //    ss << hspace << "    from: n/a" << std::endl;
    //}
    //if(getTargetVertex())
    //{
    //    ss << hspace << "    to: " << getTargetVertex()->toString() << std::endl;
    //} else {
    //    ss << hspace << "    to: n/a" << std::endl;
    //}
    ss << hspace << "    providers: " << std::endl;
    ss << Role::toString(mProviders, indent + 8);
    ss << hspace << "    consumers: " << mUsedCapacity.size() << std::endl;
    for(const std::pair<const Role, uint32_t>& consumer : mUsedCapacity)
    {
        ss << hspace << "        - " << consumer.first.toString() << ": " << consumer.second << std::endl;
    }
    return ss.str();
}

void CapacityLink::addProvider(const Role& role, uint32_t capacity)
{
    Role::Set::const_iterator cit = std::find(mProviders.begin(), mProviders.end(), role);
    if(cit != mProviders.end())
    {
        throw std::invalid_argument("templ::CapacityLink::addProvider: provider '" + role.toString() + "'"
                "does already exist for this link");
    }

    mProviders.insert(role);

    if(capacity == std::numeric_limits<uint32_t>::max() || mMaxCapacity == std::numeric_limits<uint32_t>::max())
    {
        mMaxCapacity = std::numeric_limits<uint32_t>::max();
    } else {
        mMaxCapacity += capacity;
    }
}

void CapacityLink::addConsumer(const Role& role, uint32_t capacity)
{
    if( mUsedCapacity.find(role) != mUsedCapacity.end())
    {
        throw std::invalid_argument("templ::CapacityLink::addConsumer: consumer '" + role.toString() + "'"
                "does already exist for this link");
    }
    mUsedCapacity[role] = capacity;
}

double CapacityLink::getConsumptionLevel() const
{
    if(mMaxCapacity == 0)
    {
        return 1.0;
    } else {
        return getUsedCapacity() / (1.0*mMaxCapacity);
    }
}

uint32_t CapacityLink::getRemainingCapacity() const
{
    return mMaxCapacity - getUsedCapacity();
}

uint32_t CapacityLink::getUsedCapacity() const
{
    uint32_t capacity = 0;
    std::map<Role, uint32_t>::const_iterator cit = mUsedCapacity.begin();
    for(; cit != mUsedCapacity.end(); ++cit)
    {
        capacity += cit->second;
    }
    return capacity;
}

const Role::Set& CapacityLink::getAllRoles() const
{
    if(mAllRoles.empty() && !(mProviders.empty() && mUsedCapacity.empty()))
    {
        for(const Role& r : mProviders)
        {
            if(r != msLocationTransitionRole)
            {
                mAllRoles.insert(r);
            }
        }

        for(const std::pair<const Role, uint32_t>& p : mUsedCapacity)
        {
            mAllRoles.insert(p.first);
        }
    }

    return mAllRoles;
}

std::string CapacityLink::serializeProviders() const
{
    std::stringstream ss;
    boost::archive::text_oarchive oarch(ss);
    oarch << mProviders;
    return ss.str();
}

void CapacityLink::deserializeProviders(const std::string& data)
{
    std::stringstream ss;
    ss << data;
    boost::archive::text_iarchive iarch(ss);
    iarch >> mProviders;
}

std::string CapacityLink::serializeConsumers() const
{
    std::stringstream ss;
    boost::archive::text_oarchive oarch(ss);
    oarch << mUsedCapacity;
    return ss.str();
}

void CapacityLink::deserializeConsumers(const std::string& data)
{
    std::stringstream ss;
    ss << data;
    boost::archive::text_iarchive iarch(ss);
    iarch >> mUsedCapacity;
}

std::string CapacityLink::serializeMaxCapacity() const
{
    std::stringstream ss;
    ss << mMaxCapacity;
    return ss.str();
}

void CapacityLink::deserializeMaxCapacity(const std::string& data)
{
    std::stringstream ss(data);
    ss >> mMaxCapacity;
}

void CapacityLink::registerAttributes(graph_analysis::EdgeTypeManager* eManager) const
{
    eManager->registerAttribute(getClassName(), "max_capacity",
            (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&CapacityLink::serializeMaxCapacity,
            (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&CapacityLink::deserializeMaxCapacity,
            (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&CapacityLink::serializeMaxCapacity);

    eManager->registerAttribute(getClassName(), "providers",
            (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&CapacityLink::serializeProviders,
            (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&CapacityLink::deserializeProviders,
            (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&CapacityLink::serializeProviders);

    eManager->registerAttribute(getClassName(), "consumers",
            (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&CapacityLink::serializeConsumers,
            (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&CapacityLink::deserializeConsumers,
            (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&CapacityLink::serializeConsumers);
}

} // end namespace templ
