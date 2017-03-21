#include <templ/io/MissionReader.hpp>

#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <base-logging/Logging.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/utils/XMLUtils.hpp>
#include <templ/utils/CartographicMapping.hpp>
#include <owlapi/vocabularies/XSD.hpp>

using namespace templ::utils;

namespace templ {
namespace io {

Mission MissionReader::fromFile(const std::string& url, const organization_model::OrganizationModel::Ptr& om)
{
    Mission mission(om);
    mission.setScenarioFile(url);

    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION

    // The resulting document tree
    xmlDocPtr doc;

    xmlParserOption options =  XML_PARSE_NOENT; // http://xmlsoft.org/html/libxml-parser.html#xmlParserOption

    try {
        // xmlReadFile can take filename or url
        doc = xmlReadFile(url.c_str(), NULL, options);
        if(doc == NULL)
        {
            throw std::runtime_error("templ::io::MissionReader::fromFile: Failed to parse url '" + url + "'");
        }

        xmlNodePtr rootNode;
        rootNode = xmlDocGetRootElement(doc);
        if(rootNode == NULL)
        {
            throw std::invalid_argument("templ::io::MissionReader::fromFile: Empty document");
        }

        if(xmlStrcmp(rootNode->name, (const xmlChar*) "mission"))
        {
            throw std::invalid_argument("templ::io::MissionReader::fromFile: Unexpected root node type: '" + std::string((const char*) rootNode->name) + "' -- expected <mission> tag");
        }
        LOG_INFO_S << "Found root node: " << rootNode->name;

        xmlNodePtr firstLevelChild = rootNode->xmlChildrenNode;
        while(firstLevelChild != NULL)
        {
            if(XMLUtils::nameMatches(firstLevelChild, "name"))
            {
                std::string name = XMLUtils::getContent(doc, firstLevelChild);
                LOG_DEBUG_S << "Found first level node: 'name' " << name;
                mission.setName(name);
            } else if(XMLUtils::nameMatches(firstLevelChild, "resources"))
            {
                LOG_DEBUG_S << "Found first level node: 'resources' ";
                organization_model::ModelPool modelPool = parseResources(doc, firstLevelChild);
                mission.setAvailableResources(modelPool);
            } else if(XMLUtils::nameMatches(firstLevelChild, "requirements"))
            {
                LOG_DEBUG_S << "Found first level node: 'requirements' ";
                std::vector<SpatioTemporalRequirement> requirements = parseRequirements(doc, firstLevelChild);
                std::vector<SpatioTemporalRequirement>::const_iterator cit = requirements.begin();
                for(;cit != requirements.end(); ++cit)
                {
                    const SpatioTemporalRequirement& requirement = *cit;

                    SpatioTemporalRequirement::Ptr requirementPtr(new SpatioTemporalRequirement(requirement));

                    std::string locationId = requirement.spatial.location.id;
                    symbols::Constant::Ptr constant = mission.getConstant(locationId, symbols::Constant::LOCATION);
                    symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(constant);
                    assert(location);

                    using namespace solvers::temporal::point_algebra;

                    solvers::temporal::TemporalConstraintNetwork::Ptr tcn = mission.getTemporalConstraintNetwork();
                    TimePoint::Ptr from = tcn->getOrCreateTimePoint(requirement.temporal.from);
                    TimePoint::Ptr to = tcn->getOrCreateTimePoint(requirement.temporal.to);

                    if( from->getType() != to->getType())
                    {
                        throw std::invalid_argument("templ::io::MissionReader::fromFile: temporal definition mixes qualitative"
                                " and quantitative values: from '" + requirement.temporal.from + "' "
                                " and to '" + requirement.temporal.to + "'");
                    }

                    LOG_WARN_S << "Handle: requirement: " << requirement.toString();
                    std::vector<ResourceRequirement>::const_iterator cit = requirement.resources.begin();
                    for(; cit != requirement.resources.end(); ++cit)
                    {
                        const ResourceRequirement& resource = *cit;
                        {
                            // setting the min cardinality by default
                            solvers::temporal::TemporalAssertion::Ptr temporalAssertion = mission.addResourceLocationCardinalityConstraint(location, from, to,
                                    resource.model,
                                    resource.minCardinality,
                                    owlapi::model::OWLCardinalityRestriction::MIN);
                            // Keep track for explanation
                            // TODO: we should add that to an ontology
                            mission.addRelation(temporalAssertion, "inducedBy", requirementPtr);

                        }

                        {
                            // setting the max cardinality
                            solvers::temporal::TemporalAssertion::Ptr temporalAssertion = mission.addResourceLocationCardinalityConstraint(location, from, to,
                                    resource.model,
                                    resource.maxCardinality,
                                    owlapi::model::OWLCardinalityRestriction::MAX);
                            // Keep track for explanation
                            mission.addRelation(temporalAssertion, "inducedBy", requirementPtr);
                        }

                        if(!resource.numericAttributeRequirements.empty())
                        {
                            NumericAttributeRequirements::const_iterator cit = resource.numericAttributeRequirements.begin();
                            for(; cit != resource.numericAttributeRequirements.end(); ++cit)
                            {
                                const NumericAttributeRequirement& nar = *cit;
                                mission.addResourceLocationNumericAttributeConstraint(location,
                                        from,
                                        to,
                                        resource.model,
                                        nar.model,
                                        nar.minInclusive,
                                        nar.maxInclusive
                                );
                            }
                        }

                    }
                }
            } else if(XMLUtils::nameMatches(firstLevelChild, "constraints"))
            {
                LOG_DEBUG_S << "Found first level node: 'constraints' ";
                Constraints constraints = XMLUtils::parseConstraints(doc, firstLevelChild);

                std::vector<TemporalConstraint>::const_iterator cit = constraints.temporal.begin();
                for(; cit != constraints.temporal.end(); ++cit)
                {
                    TemporalConstraint temporalConstraint = *cit;

                    using namespace solvers::temporal::point_algebra;
                    solvers::temporal::TemporalConstraintNetwork::Ptr tcn = mission.getTemporalConstraintNetwork();
                    TimePoint::Ptr t0 = tcn->getOrCreateTimePoint(temporalConstraint.lval);
                    TimePoint::Ptr t1 = tcn->getOrCreateTimePoint(temporalConstraint.rval);

                    mission.addTemporalConstraint(t0, t1, temporalConstraint.type);
                }
            } else if(XMLUtils::nameMatches(firstLevelChild, "constants"))
            {
                LOG_DEBUG_S << "Found first level node: 'constants' ";
                using namespace ::templ::symbols;
                std::set<Constant::Ptr> constants = parseConstants(doc, firstLevelChild);

                std::set<Constant::Ptr>::const_iterator cit = constants.begin();
                for(; cit != constants.end(); ++cit)
                {
                    mission.addConstant(*cit);
                }
            }

            firstLevelChild = firstLevelChild->next;
        }

    } catch(const std::exception& e)
    {
        xmlFreeDoc(doc);
        xmlCleanupParser();
        throw;
    }

    /*
     * Cleanup function for the XML library.
     */
    xmlCleanupParser();

    mission.validateAvailableResources();
    return mission;
}

std::pair<owlapi::model::IRI, size_t> MissionReader::parseResource(xmlDocPtr doc, xmlNodePtr current)
{
    if(XMLUtils::nameMatches(current, "resource"))
    {
        LOG_INFO_S << "Parsing: " << current->name;
        std::string model = XMLUtils::getSubNodeContent(doc, current, "model");
        std::string maxCardinalityTxt = XMLUtils::getSubNodeContent(doc, current, "maxCardinality");

        //model = "http://www.rock-robotics.org/2014/01/om-schema#Payload";

        owlapi::model::IRI modelIRI(model);
        int32_t cardinality = ::boost::lexical_cast<int32_t>(maxCardinalityTxt);
        if(cardinality < 0)
        {
            throw std::invalid_argument("templ::MissionReader::parseResource: invalid value in mission specification --"
                    " maxCardinality for '" + model + "' was '" + maxCardinalityTxt + "' but expected a value >= 0");
        }

        uint32_t maxCardinality = ::boost::lexical_cast<uint32_t>(maxCardinalityTxt);
        return std::pair<owlapi::model::IRI, size_t>(modelIRI, maxCardinality);
    }
    throw std::invalid_argument("templ::io::MissionReader::parseResource: expected tag 'resource' found '" + std::string((const char*) current->name) + "'");
}

organization_model::ModelPool MissionReader::parseResources(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_INFO_S << "Parsing: " << current->name;

    organization_model::ModelPool pool;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current, "resource"))
        {
            std::pair<owlapi::model::IRI, size_t> resourceBound = parseResource(doc, current);

            organization_model::ModelPool::const_iterator cit = pool.find(resourceBound.first);
            if(cit != pool.end())
            {
                throw std::invalid_argument("templ::io::MissionReader::parseResources: multiple resource entry of type '" +resourceBound.first.toString() + "'");
            } else {
                pool.insert(resourceBound);
            }
        }
        current = current->next;
    }
    return pool;
}

SpatialRequirement MissionReader::parseSpatialRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    SpatialRequirement requirement;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current, "location"))
        {
            requirement.location.id = XMLUtils::getSubNodeContent(doc, current, "id");
        }
        current = current->next;
    }
    return requirement;
}

std::vector<ResourceRequirement> MissionReader::parseResourceRequirements(xmlDocPtr doc, xmlNodePtr current)
{
    std::vector<ResourceRequirement> requirements;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current, "resource"))
        {
            ResourceRequirement resourceRequirement;
            resourceRequirement.model = XMLUtils::getSubNodeContent(doc, current, "model");

            try {
                std::string minCardinalityTxt = XMLUtils::getSubNodeContent(doc, current, "minCardinality");
                resourceRequirement.minCardinality = ::boost::lexical_cast<uint32_t>(minCardinalityTxt);
            } catch(...)
            {
                std::stringstream ss;
                ss <<  "templ::io::MissionReader::parseResourceRequirements: ";
                ss << "missing 'minCardinality' in resource at line: " << xmlGetLineNo(current);
                throw std::invalid_argument(ss.str());
            }

            try {
                std::string maxCardinalityTxt = XMLUtils::getSubNodeContent(doc, current, "maxCardinality");
                resourceRequirement.maxCardinality = ::boost::lexical_cast<uint32_t>(maxCardinalityTxt);
            } catch(...)
            {
                // no maxCardinality requirement available
                resourceRequirement.maxCardinality = std::numeric_limits<uint32_t>::max();
            }

            resourceRequirement.numericAttributeRequirements = parseAttributes(doc, current);
            requirements.push_back(resourceRequirement);
        }
        current = current->next;
    }
    return requirements;
}

NumericAttributeRequirements MissionReader::parseAttributes(xmlDocPtr doc, xmlNodePtr current)
{
    NumericAttributeRequirements numericAttributeRequirements;

    xmlNodePtr subNode = current->xmlChildrenNode;
    while(subNode != NULL)
    {
        if( XMLUtils::nameMatches(subNode, "attributes") )
        {
            xmlNodePtr numericAttributeNode = subNode->xmlChildrenNode;
            while(numericAttributeNode != NULL)
            {
                if( XMLUtils::nameMatches(numericAttributeNode, "attribute") )
                {
                    NumericAttributeRequirement requirement;
                    requirement.model = XMLUtils::getProperty(numericAttributeNode, "name");

                    xmlNodePtr restrictionNode = numericAttributeNode->xmlChildrenNode;
                    while(restrictionNode != NULL)
                    {
                        if( XMLUtils::nameMatches(restrictionNode, owlapi::vocabulary::XSD::restriction().toString(), true) )
                        {
                            xmlNodePtr actualRestriction = restrictionNode->xmlChildrenNode;
                            while(actualRestriction != NULL)
                            {
                                if(XMLUtils::nameMatches(actualRestriction, owlapi::vocabulary::XSD::minInclusive().toString(), true))
                                {
                                    std::string minInclusiveTxt = XMLUtils::getContent(doc,actualRestriction);
                                    requirement.minInclusive = ::boost::lexical_cast<int32_t>(minInclusiveTxt);
                                } else if( XMLUtils::nameMatches(actualRestriction, owlapi::vocabulary::XSD::maxInclusive().toString(), true) )
                                {
                                    std::string maxInclusiveTxt = XMLUtils::getContent(doc,actualRestriction);
                                    requirement.maxInclusive = ::boost::lexical_cast<int32_t>(maxInclusiveTxt);
                                }
                                actualRestriction = actualRestriction->next;
                            }
                        }

                        restrictionNode = restrictionNode->next;
                    }

                    LOG_DEBUG_S << "Add requirement: " << requirement.toString();
                    numericAttributeRequirements.push_back(requirement);
                }
                numericAttributeNode = numericAttributeNode->next;
            }
        }
        subNode = subNode->next;
    }
    return numericAttributeRequirements;
}

ResourceReificationRequirement MissionReader::parseResourceReificationRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    ResourceReificationRequirement requirement;

    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current, "resource"))
        {
            std::string model = XMLUtils::getSubNodeContent(doc, current, "model");
            std::string ids;
            try {
                ids = XMLUtils::getSubNodeContent(doc, current, "ids");
            } catch(...)
            {
                // no id constraint present
            }

            if(!ids.empty())
            {
                std::vector<std::string> listOfIds;
                ::boost::split(listOfIds, ids, boost::is_any_of(",;"));

                std::string minCardinalityTxt = XMLUtils::getSubNodeContent(doc, current, "minCardinality");
                uint32_t minCardinality = ::boost::lexical_cast<uint32_t>(minCardinalityTxt);
                if(minCardinality < listOfIds.size())
                {
                    throw std::invalid_argument("templ::io::MissionReader: invalid specification of reificiation constraint "
                            "minCardinality of model '" + model + "' must be at minimum of same size as the id list");
                }

                std::vector<std::string>::const_iterator cit = listOfIds.begin();
                for(; cit != listOfIds.end(); ++cit)
                {
                    uint32_t id = ::boost::lexical_cast<uint32_t>(*cit);

                    ResourceReification reification(model,id);
                    requirement.reifications.push_back(reification);
                    LOG_DEBUG_S << "Adding reification requirement" << model << " id: " << id;

                }
            }
        }
        current = current->next;
    }

    return requirement;
}


SpatioTemporalRequirement MissionReader::parseRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    SpatioTemporalRequirement requirement;
    if(XMLUtils::nameMatches(current, "requirement"))
    {
        LOG_INFO_S << "Parsing: " << current->name;
        std::string id = XMLUtils::getProperty(current, "id");
        requirement.id = boost::lexical_cast<uint32_t>(id);

        xmlNodePtr requirementNode = current->xmlChildrenNode;
        while(requirementNode != NULL)
        {
            if(XMLUtils::nameMatches(requirementNode, "spatial-requirement"))
            {
                LOG_DEBUG_S << "Parse spatial requirement";
                requirement.spatial = parseSpatialRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed spatial requirement: " << requirement.spatial.toString();

            } else if(XMLUtils::nameMatches(requirementNode, "temporal-requirement"))
            {
                LOG_DEBUG_S << "Parse temporal requirement";
                requirement.temporal = XMLUtils::parseTemporalRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed temporal requirement: " << requirement.temporal.toString();
            } else if(XMLUtils::nameMatches(requirementNode, "resource-requirement"))
            {
                LOG_DEBUG_S << "Parse resources requirement";
                requirement.resources = parseResourceRequirements(doc, requirementNode);
                LOG_WARN_S << "Parsed resources requirement";
            }
            requirementNode = requirementNode->next;
        }
    } else {
        throw std::invalid_argument("templ::io::MissionReader::parseRequirement: Unexpected tag: '" + std::string((const char*) current->name) + "' expected requirement");
    }
    return requirement;
}

std::vector<SpatioTemporalRequirement> MissionReader::parseRequirements(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_INFO_S << "Parsing: " << current->name;
    std::vector<SpatioTemporalRequirement> requirements;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current, "requirement"))
        {
            SpatioTemporalRequirement requirement = parseRequirement(doc, current);
            LOG_INFO_S << "Parsed requirement: " << requirement.toString();
            requirements.push_back(requirement);
        }
        current = current->next;
    }
    return requirements;
}

std::set<templ::symbols::Constant::Ptr> MissionReader::parseConstants(xmlDocPtr doc, xmlNodePtr current)
{
    std::set<templ::symbols::Constant::Ptr> constants;
    std::set<std::string> locations;

    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(XMLUtils::nameMatches(current, "location"))
        {
            std::string name = XMLUtils::getSubNodeContent(doc, current, "id");

            if(locations.count(name))
            {
                throw std::invalid_argument("templ::io::MissionReader::parseConstants: location '" + name + "' defined"
                        " multiple times");
            }
            base::Point position;
            bool metricDefinition = false;
            try {
                position.x() = ::boost::lexical_cast<int32_t>( XMLUtils::getSubNodeContent(doc, current, "x") );
                position.y() = ::boost::lexical_cast<int32_t>( XMLUtils::getSubNodeContent(doc, current, "y") );
                position.z() = ::boost::lexical_cast<int32_t>( XMLUtils::getSubNodeContent(doc, current, "z") );
                metricDefinition = true;
            } catch(const std::exception& e)
            {
                LOG_DEBUG_S << "Failed to extract metric location information: " << e.what();
            }

            if(!metricDefinition)
            {
                try {

                    double latitude = ::boost::lexical_cast<double>( XMLUtils::getSubNodeContent(doc, current, "latitude") );
                    double longitude = ::boost::lexical_cast<double>( XMLUtils::getSubNodeContent(doc, current, "longitude") );
                    std::string radius = XMLUtils::getSubNodeContent(doc, current, "radius");
                    utils::CartographicMapping mapping(radius);
                    base::Point point(latitude, longitude, 0.0);
                    position = mapping.latitudeLongitudeToMetric(point);
                    LOG_INFO_S << "LatitudeLongitude information: "
                        << "x: " << point.x()
                        << " , y: " << point.y()
                        << " , z: " << point.z();
                } catch(const std::exception& e)
                {
                    throw std::runtime_error("templ::io::MissionReader::parseConstants: failed to extract location: " + std::string(e.what()));
                }
            }

            LOG_INFO_S << "Metric location information: "
                << "x: " << position.x()
                << " , y: " << position.y()
                << " , z" << position.z();

            using namespace ::templ::symbols;
            constants::Location::Ptr location(new constants::Location( name, position));
            constants.insert(location);
            locations.insert(name);
        }
        current = current->next;
    }
    return constants;
}

} // end namespace io
} // end namespace templ

