#include "MissionRequirements.hpp"

namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace io {

std::string SpatialRequirement::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "SpatialRequirement:" << std::endl;
    ss << hspace << "    " << location.toString() << std::endl;
    return ss.str();
}

std::string TemporalRequirement::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << "TemporalRequirement:" << std::endl;
    ss << "    from: " << from << std::endl;
    ss << "    to:   " << to << std::endl;
    return ss.str();
}

std::string NumericAttributeRequirement::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "NumericAttribute: " << model.toString() << std::endl;
    ss << hspace << "    minInclusive: " << minInclusive << std::endl;
    ss << hspace << "    maxInclusive: " << maxInclusive << std::endl;
    return ss.str();
}

std::string NumericAttributeRequirement::toString(const std::vector<NumericAttributeRequirement>& nar, uint32_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;

    std::vector<NumericAttributeRequirement>::const_iterator cit = nar.begin();
    for(; cit != nar.end(); ++cit)
    {
        ss << cit->toString(indent) << std::endl;
    }
    return ss.str();
}

ResourceRequirement::ResourceRequirement()
    : minCardinality(std::numeric_limits<uint32_t>::min())
    , maxCardinality(std::numeric_limits<uint32_t>::max())
{}

std::string ResourceRequirement::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << model.toString() << std::endl;
    ss << hspace << "    minCardinality: " << minCardinality << std::endl;
    ss << hspace << "    maxCardinality: " << maxCardinality << std::endl;
    ss << NumericAttributeRequirement::toString(numericAttributeRequirements, indent + 4) << std::endl;
    return ss.str();
}

std::string ResourceReificationRequirement::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "ResourceReificationRequirement:" << std::endl;
    std::vector<ResourceReification>::const_iterator cit = reifications.begin();
    for(; cit != reifications.end(); ++cit)
    {
        ss << hspace << "    " << cit->first.toString() << " " << cit->second << std::endl;
    }
    return ss.str();
}

SpatioTemporalRequirement::SpatioTemporalRequirement()
    : graph_analysis::Vertex()
{}

SpatioTemporalRequirement::~SpatioTemporalRequirement()
{}

std::string SpatioTemporalRequirement::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << std::endl << hspace << "SpatioTemporalRequirement (id:" << id << ")" << std::endl;
    ss << spatial.toString(indent + 4);
    ss << temporal.toString(indent + 4);
    ss << hspace << "Resources: " << std::endl;
    std::vector<ResourceRequirement>::const_iterator cit = resources.begin();
    for(; cit != resources.end(); ++cit)
    {
        ss << cit->toString(indent + 4);
    }
    return ss.str();
}

std::string Constraints::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "Constraints:" << std::endl;
    for(const TemporalConstraint& t : temporal)
    {
        ss << t.toString(indent + 4);
    }
    for(const constraints::ModelConstraint& m : model)
    {
        ss << m.toString(indent + 4);
    }
    return ss.str();
}



} // end namespace io
} // end namespace templ
