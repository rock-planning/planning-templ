#include "CapacityLink.hpp"
#include <sstream>
#include <graph_analysis/EdgeTypeManager.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace templ {

const graph_analysis::EdgeRegistration<CapacityLink> CapacityLink::msRegistration;

CapacityLink::CapacityLink()
    : graph_analysis::Edge()
    , mProvider()
    , mMaxCapacity(0)
{}

CapacityLink::CapacityLink(const Role& role, uint32_t capacity)
    : graph_analysis::Edge()
    , mProvider(role)
    , mMaxCapacity(capacity)
{}

CapacityLink::~CapacityLink() {}

std::string CapacityLink::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "CapacityLink " << "(max: " << getCapacity() << ")" << std::endl;
    ss << hspace << "    provider: " << mProvider.toString() << std::endl;
    ss << hspace << "    consumers: " << std::endl;
    for(const std::pair<Role, uint32_t>& consumer : mUsedCapacity)
    {
        ss << hspace << "        - " << consumer.first.toString() << ": " << consumer.second << std::endl;
    }
    return ss.str();
}

void CapacityLink::addConsumer(const Role& role, uint32_t capacity)
{
    if( mUsedCapacity.find(role) != mUsedCapacity.end())
    {
        throw std::invalid_argument("templ::CapacityLink::addConsumer: consumer '" + role.toString() + "'"
                "does already exist for this link");
    }
    uint32_t remainingCapacity = getRemainingCapacity();

    if(remainingCapacity < capacity)
    {
        std::stringstream ss;
        ss << "templ::CapacitLink::addConsumer: consumer '" << role.toString() << "'"
            << "cannot be added, since there is not enough capacity left: '" << remainingCapacity << "'";
        throw std::runtime_error(ss.str());
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

std::string CapacityLink::serializeProvider() const
{
    std::stringstream ss;
    boost::archive::text_oarchive oarch(ss);
    oarch << mProvider;
    return ss.str();
}

void CapacityLink::deserializeProvider(const std::string& data)
{
    std::stringstream ss;
    ss << data;
    boost::archive::text_iarchive iarch(ss);
    iarch >> mProvider;
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

    eManager->registerAttribute(getClassName(), "provider",
            (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&CapacityLink::serializeProvider,
            (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&CapacityLink::deserializeProvider,
            (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&CapacityLink::serializeProvider);

    eManager->registerAttribute(getClassName(), "consumers",
            (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&CapacityLink::serializeConsumers,
            (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&CapacityLink::deserializeConsumers,
            (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&CapacityLink::serializeConsumers);
}

} // end namespace templ
