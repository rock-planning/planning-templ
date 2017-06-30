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
    ss << hspace << "CapacityLink" << std::endl;
    ss << hspace << "    provider: " << mProvider.toString() << std::endl;
    ss << hspace << "    users: " << std::endl;
    for(const std::pair<Role, uint32_t>& user : mUsedCapacity)
    {
        ss << hspace << "        - " << user.first.toString() << ": " << user.second << std::endl;
    }
    return ss.str();
}

void CapacityLink::addUser(const Role& role, uint32_t capacity)
{
    if( mUsedCapacity.find(role) != mUsedCapacity.end())
    {
        throw std::invalid_argument("templ::CapacityLink::addUser: user '" + role.toString() + "'"
                "does already exist for this link");
    }
    uint32_t remainingCapacity = getRemainingCapacity();

    if(remainingCapacity < capacity)
    {
        std::stringstream ss;
        ss << "templ::CapacitLink::addUser: user '" << role.toString() << "'"
            << "cannot be added, since there is not enough capacity left: '" << remainingCapacity << "'";
        throw std::runtime_error(ss.str());
    }

    mUsedCapacity[role] = capacity;
}

uint32_t CapacityLink::getRemainingCapacity()
{
    uint32_t capacity = 0;
    std::map<Role, uint32_t>::const_iterator cit = mUsedCapacity.begin();
    for(; cit != mUsedCapacity.end(); ++cit)
    {
        capacity += cit->second;
    }
    return mMaxCapacity - capacity;
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

std::string CapacityLink::serializeUsers() const
{
    std::stringstream ss;
    boost::archive::text_oarchive oarch(ss);
    oarch << mUsedCapacity;
    return ss.str();
}

void CapacityLink::deserializeUsers(const std::string& data)
{
    std::stringstream ss;
    ss << data;
    boost::archive::text_iarchive iarch(ss);
    iarch >> mUsedCapacity;
}

void CapacityLink::registerAttributes(graph_analysis::EdgeTypeManager* eManager) const
{
        eManager->registerAttribute(getClassName(), "provider",
                   (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&CapacityLink::serializeProvider,
                   (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&CapacityLink::deserializeProvider,
                   (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&CapacityLink::serializeProvider);

        eManager->registerAttribute(getClassName(), "users",
                   (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&CapacityLink::serializeUsers,
                   (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&CapacityLink::deserializeUsers,
                   (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&CapacityLink::serializeUsers);
}

} // end namespace templ
