#ifndef TEMPL_IO_MISSION_REQUIREMENTS_HPP
#define TEMPL_IO_MISSION_REQUIREMENTS_HPP

#include <owlapi/OWLApi.hpp>
#include "FluentTypes.hpp"
#include <graph_analysis/Vertex.hpp>
#include "../constraints/ModelConstraint.hpp"
#include "TemporalConstraint.hpp"

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
    size_t minCardinality;
    size_t maxCardinality;

    NumericAttributeRequirements numericAttributeRequirements;

    ResourceRequirement();

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


class SpatioTemporalRequirement : public graph_analysis::Vertex
{
public:
    typedef shared_ptr<SpatioTemporalRequirement> Ptr;

    SpatioTemporalRequirement();
    virtual ~SpatioTemporalRequirement();

    virtual std::string getClassName() const { return "SpatioTemporalRequirement"; }

    virtual std::string toString() const { return toString(0); }

    virtual std::string toString(uint32_t indent) const;

    uint32_t id;
    SpatialRequirement spatial;
    TemporalRequirement temporal;
    std::vector<ResourceRequirement> resources;

protected:
    /// Make sure cloning works
    virtual graph_analysis::Vertex* getClone() const { return new SpatioTemporalRequirement(*this); }
};

struct Constraints
{
    std::vector<TemporalConstraint> temporal;
    constraints::ModelConstraint::List model;

    std::string toString(size_t indent = 0) const;
};

} // end namespace io
} // end namespace templ
#endif // TEMPL_IO_MISSION_REQUIREMENTS_HPP
