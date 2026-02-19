#ifndef TEMPL_BENCHMARK_VRPPROBLEM_HPP
#define TEMPL_BENCHMARK_VRPPROBLEM_HPP

#include <map>
#include <vector>
#include <string>
#include <cstdint>

namespace templ {
namespace benchmark {

struct Coord2D
{
    Coord2D(double x = 0.0, double y = 0.0)
        : x(x)
        , y(y)
    {}

    double x;
    double y;

    bool operator<(const Coord2D& other) const
    {
        if(x == other.x)
        {
            return y < other.y;
        }
        return x < other.x;
    }
};

/**
 *
 * \see http://vrp.atd-lab.inf.puc-rio.br/index.php/en/
 */
class VRPProblem
{
public:
    typedef std::vector<Coord2D> NodeCoordinates;
    typedef std::vector<uint32_t> Demands;
    typedef std::vector<Coord2D> Depots;

    enum Type { CVRP };
    enum EdgeWeightType { FUNCTION, W_EUC_2D };
    enum EdgeWeightFormat { EUC_2D };
    enum NodeCoordType { TWOD_COORDS };

    static std::map<Type, std::string> TypeTxt;
    static std::map<EdgeWeightType, std::string> EdgeWeightTypeTxt;
    static std::map<EdgeWeightFormat, std::string> EdgeWeightFormatTxt;
    static std::map<NodeCoordType, std::string> NodeCoordTypeTxt;



    const std::string& getName() const { return mName; }
    void setName(const std::string& name) { mName = name; }

    const Type& getType() const { return mType; }
    void setType(Type type) { mType = type; }
    void setType(const std::string&);

    const std::string& getComment() const { return mComment; }
    void setComment(const std::string& comment) { mComment = comment; }

    uint32_t getDimension() const { return mDimension; }
    void setDimension(uint32_t dim) { mDimension = dim; }

    uint32_t getCapacity() const { return mCapacity; }
    void setCapacity(uint32_t cap) { mCapacity = cap; }

    double getDistance() const { return mDistance; }
    void setDistance(double distance) { mDistance = distance; }

    uint32_t getVehicles() const { return mVehicles; }
    void setVehicles(uint32_t v) { mVehicles = v; }

    EdgeWeightType getEdgeWeightType() const { return mEdgeWeightType; }
    void setEdgeWeightType(EdgeWeightType t) { mEdgeWeightType = t; }
    void setEdgeWeightType(const std::string&);

    EdgeWeightFormat getEdgeWeightFormat() const { return mEdgeWeightFormat; }
    void setEdgeWeightFormat(EdgeWeightFormat t) { mEdgeWeightFormat = t; }
    void setEdgeWeightFormat(const std::string&);

    NodeCoordType getNodeCoordType() const { return mNodeCoordType; }
    void setNodeCoordType(NodeCoordType t) { mNodeCoordType = t; }
    void setNodeCoordType(const std::string&);

    const NodeCoordinates& getNodeCoordinates() const { return mNodeCoordinates; }
    const Demands& getDemands() const { return mDemands; }
    uint32_t getTotalDemand() const;
    const Depots& getDepots() const { return mDepots; }

    void setNodeCoordinates(const NodeCoordinates& c) { mNodeCoordinates = c; }
    void setDemands(const Demands& d) { mDemands = d; }
    void setDepots(const Depots& d) { mDepots = d; }

    std::string toString(size_t indent = 0) const;
protected:
    std::string mName;
    Type mType;
    std::string mComment;
    uint32_t mDimension;
    uint32_t mCapacity;
    double mDistance;
    uint32_t mVehicles;

    EdgeWeightType mEdgeWeightType;
    EdgeWeightFormat mEdgeWeightFormat;

    NodeCoordType mNodeCoordType;

    NodeCoordinates mNodeCoordinates;
    Demands mDemands;
    Depots mDepots;




};

} // end namespace benchmark
} // end namespace templ
#endif // TEMPL_BENCHMARKS_VRPPROBLEM_HPP
