#ifndef TEMPL_IO_MISSION_READER_HPP
#define TEMPL_IO_MISSION_READER_HPP

#include <stdio.h>
#include <string>
#include <stdexcept>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <templ/Mission.hpp>

namespace templ {
namespace io {

struct Location
{
    std::string id;

    std::string toString() const;
};

struct SpatialRequirement
{
    Location location;

    std::string toString() const;
};

struct TemporalRequirement
{
    std::string from;
    std::string to;

    std::string toString() const;
};

struct ServiceRequirement
{
    owlapi::model::IRIList services;

    std::string toString() const;
};

struct Requirement
{
    SpatialRequirement spatial;
    TemporalRequirement temporal;
    ServiceRequirement functional;
    organization_model::ModelPool resources;

    std::string toString() const;
};

struct TemporalConstraint
{
    templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type type;

    std::string rval;
    std::string lval;

    static templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type getTemporalConstraintType(const std::string& name);

    std::string toString() const;
};

struct Constraints
{
    std::vector<TemporalConstraint> temporal;

    std::string toString() const;
};

class MissionReader
{
public:
    static Mission fromFile(const std::string& url);

private:
    static bool nameMatches(xmlNodePtr node, const std::string& name);

    static std::string getContent(xmlDocPtr doc, xmlNodePtr node, size_t count = 1);

    static std::string getProperty(xmlNodePtr node, const std::string& name);

    static std::string getSubNodeContent(xmlDocPtr doc, xmlNodePtr node, const std::string& name);

    static std::pair<owlapi::model::IRI, size_t> parseResource(xmlDocPtr doc, xmlNodePtr current);

    static organization_model::ModelPool parseResources(xmlDocPtr doc, xmlNodePtr current);

    static SpatialRequirement parseSpatialRequirement(xmlDocPtr doc, xmlNodePtr current);

    static TemporalRequirement parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current);

    static ServiceRequirement parseServiceRequirement(xmlDocPtr doc, xmlNodePtr current);

    static organization_model::ModelPool parseResourceRequirement(xmlDocPtr doc, xmlNodePtr current);

    static Requirement parseRequirement(xmlDocPtr doc, xmlNodePtr current);

    static std::vector<Requirement> parseRequirements(xmlDocPtr doc, xmlNodePtr current);

    static std::vector<TemporalConstraint> parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current);

    static Constraints parseConstraints(xmlDocPtr doc, xmlNodePtr current);
};

} // end namespace io
} // end namespace templ

#endif // TEMPL_IO_MISSION_READER_HPP
