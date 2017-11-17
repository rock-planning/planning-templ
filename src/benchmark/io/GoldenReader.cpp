#include "GoldenReader.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

namespace templ {
namespace benchmark {
namespace io {

VRPProblem GoldenReader::read(const std::string& file)
{
    VRPProblem vrp;

    // COORD_SECTION
    // DEMAND_SECTION
    // DEPOT_SECTION
    //
    std::ifstream infile(file);
    std::string line;
    std::string delimiter(":");
    while(std::getline(infile, line))
    {

        line.erase(line.find_last_not_of("\n\r\t")+1);
        std::string keyword = line.substr(0, line.find(delimiter));
        std::string value = line.substr(line.find(" ") + 1);
        if(keyword == "NAME")
        {
            vrp.setName(value);
        } else if(keyword == "TYPE")
        {
            vrp.setType(value);
        } else if(keyword == "COMMENT")
        {
            vrp.setComment(value);
        } else if(keyword == "DIMENSION")
        {
            vrp.setDimension( boost::lexical_cast<uint32_t>(value) );
        } else if(keyword == "CAPACITY")
        {
            vrp.setCapacity( boost::lexical_cast<uint32_t>(value) );
        } else if(keyword == "DISTANCE")
        {
            vrp.setDistance( boost::lexical_cast<double>(value) );
        } else if(keyword == "VEHICLES")
        {
            vrp.setVehicles( boost::lexical_cast<uint32_t>(value) );
        } else if(keyword == "EDGE_WEIGHT_TYPE")
        {
            vrp.setEdgeWeightType(value);
        } else if(keyword == "EDGE_WEIGHT_FORMAT")
        {
            vrp.setEdgeWeightFormat(value);
        } else if(keyword == "NODE_COORD_TYPE")
        {
            vrp.setNodeCoordType(value);
        } else if(keyword == "NODE_COORD_SECTION")
        {
            loadNodeCoordinates(infile, vrp);
        } else if(keyword == "DEMAND_SECTION")
        {
            loadDemands(infile, vrp);
        } else if(keyword == "DEPOT_SECTION")
        {
            loadDepots(infile, vrp);
        }
    }

    return vrp;
}

void GoldenReader::loadNodeCoordinates(std::ifstream& infile, VRPProblem& vrp)
{
    VRPProblem::NodeCoordinates coordinates;
    std::string line;
    while(true)
    {
        int streampos = infile.tellg();
        std::getline(infile, line);
        line.erase(line.find_last_not_of("\n\r\t")+1);

        std::vector<std::string> tokens;
        boost::split(tokens, line, boost::is_any_of(" "));
        if(tokens.size() == 1)
        {
            infile.seekg(streampos);
            break;
        }
        Coord2D coord( boost::lexical_cast<double>(tokens[1]),
                boost::lexical_cast<double>(tokens[2]) );
        coordinates.push_back(coord);
    }

    vrp.setNodeCoordinates(coordinates);
}

void GoldenReader::loadDepots(std::ifstream& infile, VRPProblem& vrp)
{
    VRPProblem::Depots coordinates;
    std::string line;
    while(true)
    {
        int streampos = infile.tellg();
        std::getline(infile, line);
        line.erase(line.find_last_not_of("\n\r\t")+1);

        std::vector<std::string> tokens;
        boost::split(tokens, line, boost::is_any_of(" "));
        if(tokens.size() == 1)
        {
            infile.seekg(streampos);
            break;
        }
        Coord2D coord( boost::lexical_cast<double>(tokens.at(0)),
                boost::lexical_cast<double>(tokens.at(1)) );
        coordinates.push_back(coord);
    }

    vrp.setDepots(coordinates);
}

void GoldenReader::loadDemands(std::ifstream& infile, VRPProblem& vrp)
{
    VRPProblem::Demands demands;
    std::string line;
    while(true)
    {
        int streampos = infile.tellg();
        std::getline(infile, line);
        line.erase(line.find_last_not_of("\n\r\t")+1);

        std::vector<std::string> tokens;
        boost::split(tokens, line, boost::is_any_of(" "));
        if(tokens.size() == 1)
        {
            infile.seekg(streampos);
            break;
        }
        demands.push_back(boost::lexical_cast<uint32_t>(tokens[1]));
    }

    vrp.setDemands(demands);
}

} // end namespace io
} // end namespace benchmark
} // end namespace templ
