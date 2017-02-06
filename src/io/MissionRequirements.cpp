#include "MissionRequirements.hpp"

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

templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type TemporalConstraint::getTemporalConstraintType(const std::string& name)
{
    using namespace templ::solvers::temporal::point_algebra;
    //return Empty, Greater, Less, Equal, Distinct, GreaterOrEqual, LessOrEqual, Universal,
    if(name == "greaterThan")
    {
        return QualitativeTimePointConstraint::Greater;
    } else if(name == "lessThan")
    {
        return QualitativeTimePointConstraint::Less;
    } else if(name == "equals")
    {
        return QualitativeTimePointConstraint::Equal;
    } else if(name == "distinct")
    {
        return QualitativeTimePointConstraint::Distinct;
    } else if(name == "greaterOrEqual")
    {
        return QualitativeTimePointConstraint::GreaterOrEqual;
    } else if(name == "lessOrEqual")
    {
        return QualitativeTimePointConstraint::LessOrEqual;
    }

    throw std::invalid_argument("templ::io::MissionReader::getTemporalConstraintType: unknown temporal constraint type: '" + name + "'");
}

std::string TemporalConstraint::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "TemporalConstraint:" << std::endl;
    ss << hspace << "    type:" << templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::TypeTxt[type];
    ss << hspace << "    lval:" << lval;
    ss << hspace << "    rval:" << rval;
    return ss.str();
}



} // end namespace io
} // end namespace templ
