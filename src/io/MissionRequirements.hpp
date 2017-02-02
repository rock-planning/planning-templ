#ifndef TEMPL_IO_MISSION_REQUIREMENTS_HPP
#define TEMPL_IO_MISSION_REQUIREMENTS_HPP

#include <owlapi/OWLApi.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>
#include "FluentTypes.hpp"

namespace templ {
namespace io {

/**
 * A SpatialRequirement defines the required location
 */
struct SpatialRequirement
{
    Location location;

    std::string toString(uint32_t indent = 0) const;
};

/**
 * The TemporalRequirement defines the required timeinterval using
 * (qualitative) timepoints
 */
struct TemporalRequirement
{
    std::string from;
    std::string to;

    std::string toString(uint32_t indent = 0) const;
};

struct NumericAttributeRequirement
{
    // Model of the attribute
    owlapi::model::IRI model;
    int32_t minInclusive;
    int32_t maxInclusive;

    std::string toString(uint32_t indent = 0) const;

    static std::string toString(const std::vector<NumericAttributeRequirement>& nar, uint32_t indent = 0);
};

typedef std::vector<NumericAttributeRequirement> NumericAttributeRequirements;

/**
 * The ResourceRequirement defines the required functionalities or actual atomic
 * agents by using IRIs
 */
struct ResourceRequirement
{
    owlapi::model::IRI model;
    int minCardinality;
    int maxCardinality;

    NumericAttributeRequirements numericAttributeRequirements;

    std::string toString(uint32_t indent = 0) const;
};

struct ResourceRequirements
{
    std::vector<ResourceRequirement> requirements;
    std::string toString(uint32_t indent = 0) const;
};

typedef std::pair< owlapi::model::IRI, uint32_t> ResourceReification;

/**
 * The ResourceReification constraint defines the required resource
 * identifcation or (partial grounding of this typed resource variable)
 */
struct ResourceReificationRequirement
{
    std::vector<ResourceReification> reifications;

    std::string toString(uint32_t indent = 0) const;
};

struct Requirement
{
    uint32_t id;
    SpatialRequirement spatial;
    TemporalRequirement temporal;
    std::vector<ResourceRequirement> resources;

    std::string toString(uint32_t indent = 0) const;
};

struct TemporalConstraint
{
    templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type type;

    std::string rval;
    std::string lval;

    static templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type getTemporalConstraintType(const std::string& name);

    std::string toString(uint32_t indent = 0) const;
};

struct Constraints
{
    std::vector<TemporalConstraint> temporal;

    std::string toString() const;
};

} // end namespace io
} // end namespace templ
#endif // TEMPL_IO_MISSION_REQUIREMENTS_HPP
