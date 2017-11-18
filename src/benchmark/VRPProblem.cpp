#include "VRPProblem.hpp"
#include <algorithm>
#include <iostream>
#include <sstream>

namespace templ {
namespace benchmark {

std::map<VRPProblem::Type, std::string> VRPProblem::TypeTxt = { { VRPProblem::CVRP, "CVRP" } };

std::map<VRPProblem::EdgeWeightType, std::string> VRPProblem::EdgeWeightTypeTxt = {
    { VRPProblem::FUNCTION, "FUNCTION"},
    { VRPProblem::W_EUC_2D, "EUC_2D"}
};

std::map<VRPProblem::EdgeWeightFormat, std::string> VRPProblem::EdgeWeightFormatTxt = {
    { VRPProblem::EUC_2D, "EUC_2D" }
};

std::map<VRPProblem::NodeCoordType, std::string> VRPProblem::NodeCoordTypeTxt = {
    { VRPProblem::TWOD_COORDS, "TWOD_COORDS" }
};

void VRPProblem::setType(const std::string& type)
{
    std::map<Type, std::string>::const_iterator cit = std::find_if(TypeTxt.begin(), TypeTxt.end(), [type](const std::pair<Type, std::string>& other)
            {
                return type == other.second;
            });
    if(cit == TypeTxt.end())
    {
        throw std::invalid_argument("VRPProblem::setFormat: '" + type + "' is not a known Type");
    }

    setType(cit->first);
}

void VRPProblem::setEdgeWeightType(const std::string& type)
{
    std::map<EdgeWeightType, std::string>::const_iterator cit = std::find_if(EdgeWeightTypeTxt.begin(), EdgeWeightTypeTxt.end(), [type](const std::pair<EdgeWeightType, std::string>& other)
            {
                return type == other.second;
            });
    if(cit == EdgeWeightTypeTxt.end())
    {
        throw std::invalid_argument("VRPProblem::setEdgeWeightFormat: '" + type + "' is not a known EdgeWeightType");
    }
    setEdgeWeightType(cit->first);
}

void VRPProblem::setNodeCoordType(const std::string& type)
{
    std::map<NodeCoordType, std::string>::const_iterator cit = std::find_if(NodeCoordTypeTxt.begin(), NodeCoordTypeTxt.end(), [type](const std::pair<NodeCoordType, std::string>& other)
            {
                return type == other.second;
            });
    if(cit == NodeCoordTypeTxt.end())
    {
        throw std::invalid_argument("VRPProblem::setNodeCoordType: '" + type + "' is not a known NodeCoordType");
    }
    setNodeCoordType(cit->first);
}

void VRPProblem::setEdgeWeightFormat(const std::string& type)
{
    std::map<EdgeWeightFormat, std::string>::const_iterator cit = std::find_if(EdgeWeightFormatTxt.begin(), EdgeWeightFormatTxt.end(), [type](const std::pair<EdgeWeightFormat, std::string>& other)
            {
                return type == other.second;
            });
    if(cit == EdgeWeightFormatTxt.end())
    {
        throw std::invalid_argument("VRPProblem::setEdgeWeightFormat: '" + type + "' is not a known EdgeWeightFormat");
    }
    setEdgeWeightFormat(cit->first);
}

std::string VRPProblem::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "NAME: " << getName() << std::endl;
    ss << hspace << "TYPE: " << TypeTxt[ getType() ] << std::endl;
    ss << hspace << "COMMENT: " << getComment() << std::endl;
    ss << hspace << "DIMENSION: " << getDimension() << std::endl;
    ss << hspace << "CAPACITY: " << getCapacity() << std::endl;
    ss << hspace << "DISTANCE: " << getDistance() << std::endl;
    ss << hspace << "VEHICLES: " << getVehicles() << std::endl;
    ss << hspace << "EDGE_WEIGHT_TYPES: " << EdgeWeightTypeTxt[ getEdgeWeightType() ] << std::endl;
    ss << hspace << "EDGE_WEIGHT_FORMAT: " << EdgeWeightFormatTxt[ getEdgeWeightFormat() ] << std::endl;
    ss << hspace << "NODE_COORD_TYPE: " << NodeCoordTypeTxt[ getNodeCoordType() ] << std::endl;
    ss << hspace << "NODE_COORDS_SECTION" << std::endl;
    size_t idx = 0;
    for(const Coord2D& c : mNodeCoordinates)
    {
        ++idx;
        ss << hspace << idx << " " << c.x << " " << c.y << std::endl;
    }
    ss << hspace << "DEMAND_SECTION" << std::endl;
    idx = 0;
    for(size_t d : mDemands)
    {
        ++idx;
        ss << hspace << idx << " " << d << std::endl;
    }
    ss << hspace << "DEPOT_SECTION" << std::endl;
    ss << hspace << mDepots[0].x << " " << mDepots.at(0).y << std::endl;
    ss << hspace << "-1" << std::endl;
    ss << hspace << "EOF" << std::endl;
    return ss.str();
}

uint32_t VRPProblem::getTotalDemand() const
{
    uint32_t demand = 0;
    for(Demands::value_type d : mDemands)
    {
        demand += d;
    }
    return demand;
}

} // end namespace benchmark
} // end namespace templ
