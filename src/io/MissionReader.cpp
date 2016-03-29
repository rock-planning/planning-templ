#include <templ/io/MissionReader.hpp>

#include <sstream>
#include <boost/lexical_cast.hpp>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <base/Logging.hpp>
#include <templ/SharedPtr.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/utils/CartographicMapping.hpp>

namespace templ {
namespace io {

std::string Location::toString() const
{
    std::stringstream ss;
    ss << "Location: " << id;
    return ss.str();
}


std::string SpatialRequirement::toString() const
{
    std::stringstream ss;
    ss << "SpatialRequirement:" << std::endl;
    ss << "    location: " << location.toString() << std::endl;
    return ss.str();
}

std::string TemporalRequirement::toString() const
{
    std::stringstream ss;
    ss << "TemporalRequirement:" << std::endl;
    ss << "    from: " << from << std::endl;
    ss << "    to:   " << to << std::endl;
    return ss.str();
}


std::string ServiceRequirement::toString() const
{
    return owlapi::model::IRI::toString(services);
}


std::string Requirement::toString() const
{
    std::stringstream ss;
    ss << "Requirement: " << std::endl;
    ss << spatial.toString();
    ss << temporal.toString();
    ss << "Services: ";
    ss << functional.toString() << std::endl;
    ss << resources.toString();
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

std::string TemporalConstraint::toString() const
{
    std::stringstream ss;
    ss << "TemporalConstraint:" << std::endl;
    ss << "    type:" << templ::solvers::temporal::point_algebra::QualitativeTimePointConstraint::TypeTxt[type];
    ss << "    lval:" << lval;
    ss << "    rval:" << rval;
    return ss.str();
}

bool MissionReader::nameMatches(xmlNodePtr node, const std::string& name)
{
    return xmlStrcmp(node->name, (const xmlChar*) name.c_str()) == 0;
}

std::string MissionReader::getContent(xmlDocPtr doc, xmlNodePtr node, size_t count)
{
    xmlChar* key = xmlNodeListGetString(doc, node->xmlChildrenNode, count);
    if(key)
    {
        std::string content((const char*) key);
        xmlFree(key);

        return content;
    } else {
        return std::string();
    }
}

std::string MissionReader::getProperty(xmlNodePtr node, const std::string& name)
{
    std::string property;
    xmlChar* xmlName = xmlCharStrdup(name.c_str());
    xmlChar* value = xmlGetProp(node, xmlName);
    xmlFree(xmlName);
    if(value)
    {
        property = std::string((const char*) value);
        xmlFree(value);
        return property;
    }
    throw std::invalid_argument("templ::io::MissionReader::getProperty: could not find property '" + name + "'");
}

std::string MissionReader::getSubNodeContent(xmlDocPtr doc, xmlNodePtr node, const std::string& name)
{
    xmlNodePtr subNode = node->xmlChildrenNode;
    while(subNode != NULL)
    {
        if(nameMatches(subNode, name))
        {
            return getContent(doc, subNode);
        }

        subNode = subNode->next;
    }
    throw std::invalid_argument("templ::io::MissionReader::getSubNodeContent: could not find subnode '" + name + "' in node '" + std::string((const char*) node->name) + "'");
}

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

    xmlParserOption options = (xmlParserOption) 0;

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
            if(nameMatches(firstLevelChild, "name"))
            {
                std::string name = getContent(doc, firstLevelChild);
                LOG_DEBUG_S << "Found first level node: 'name' " << name;
                mission.setName(name);
            } else if(nameMatches(firstLevelChild, "resources"))
            {
                LOG_DEBUG_S << "Found first level node: 'resources' ";
                organization_model::ModelPool modelPool = parseResources(doc, firstLevelChild);
                mission.setAvailableResources(modelPool);
            } else if(nameMatches(firstLevelChild, "requirements"))
            {
                LOG_DEBUG_S << "Found first level node: 'requirements' ";
                std::vector<Requirement> requirements = parseRequirements(doc, firstLevelChild);
                std::vector<Requirement>::const_iterator cit = requirements.begin();
                for(;cit != requirements.end(); ++cit)
                {
                    const Requirement& requirement = *cit;

                    std::string locationId = requirement.spatial.location.id;
                    symbols::Constant::Ptr constant = mission.getConstant(locationId, symbols::Constant::LOCATION);
                    symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(constant);
                    assert(location);

                    using namespace solvers::temporal::point_algebra;

                    TimePoint::Ptr from = mission.getOrCreateTimePoint(requirement.temporal.from);
                    TimePoint::Ptr to = mission.getOrCreateTimePoint(requirement.temporal.to);

                    if( from->getType() != to->getType())
                    {
                        throw std::invalid_argument("templ::io::MissionReader::fromFile: temporal definition mixes qualitative"
                                " and quantitative values: from '" + requirement.temporal.from + "' "
                                " and to '" + requirement.temporal.to + "'");
                    }

                    owlapi::model::IRIList::const_iterator sit = requirement.functional.services.begin();
                    for(; sit != requirement.functional.services.end(); ++sit)
                    {
                        const owlapi::model::IRI& model = *sit;
                        mission.addResourceLocationCardinalityConstraint(location, from, to, model);
                    }

                    organization_model::ModelPool::const_iterator mit = requirement.resources.begin();
                    for(; mit != requirement.resources.end(); ++mit)
                    {
                        const owlapi::model::IRI& model = mit->first;
                        uint32_t cardinality = mit->second;

                        // setting the min cardinality by default
                        mission.addResourceLocationCardinalityConstraint(location, from, to, model, cardinality);
                    }
                }
            } else if(nameMatches(firstLevelChild, "constraints"))
            {
                LOG_DEBUG_S << "Found first level node: 'constraints' ";
                Constraints constraints = parseConstraints(doc, firstLevelChild);

                std::vector<TemporalConstraint>::const_iterator cit = constraints.temporal.begin();
                for(; cit != constraints.temporal.end(); ++cit)
                {
                    TemporalConstraint temporalConstraint = *cit;

                    using namespace solvers::temporal::point_algebra;
                    TimePoint::Ptr t0 = mission.getOrCreateTimePoint(temporalConstraint.lval);
                    TimePoint::Ptr t1 = mission.getOrCreateTimePoint(temporalConstraint.rval);
                    mission.addTemporalConstraint(t0, t1, temporalConstraint.type);
                }
            } else if(nameMatches(firstLevelChild, "constants"))
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
    if(nameMatches(current, "resource"))
    {
        LOG_INFO_S << "Parsing: " << current->name;
        std::string model = getSubNodeContent(doc, current, "model");
        std::string maxCardinalityTxt = getSubNodeContent(doc, current, "maxCardinality");

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
        if(nameMatches(current, "resource"))
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
        if(nameMatches(current, "location"))
        {
            requirement.location.id = getSubNodeContent(doc, current, "id");
        }
        current = current->next;
    }
    return requirement;
}

TemporalRequirement MissionReader::parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    TemporalRequirement requirement;

    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current,"from"))
        {
            requirement.from = getContent(doc, current);
        } else if(nameMatches(current, "to"))
        {
            requirement.to = getContent(doc, current);
        }
        current = current->next;
    }
    return requirement;
}

ServiceRequirement MissionReader::parseServiceRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    ServiceRequirement requirement;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "service"))
        {
            std::string service = getContent(doc, current);
            requirement.services.push_back( owlapi::model::IRI(service) );
        }
        current = current->next;
    }
    return requirement;
}

organization_model::ModelPool MissionReader::parseResourceRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    organization_model::ModelPool modelPool;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "resource"))
        {
            std::string model = getSubNodeContent(doc, current, "model");
            std::string minCardinalityTxt = getSubNodeContent(doc, current, "minCardinality");

            uint32_t minCardinality = ::boost::lexical_cast<uint32_t>(minCardinalityTxt);
            modelPool[ owlapi::model::IRI(model) ] = minCardinality;
        }
        current = current->next;
    }

    return modelPool;
}

Requirement MissionReader::parseRequirement(xmlDocPtr doc, xmlNodePtr current)
{
    Requirement requirement;
    if(nameMatches(current, "requirement"))
    {
        LOG_INFO_S << "Parsing: " << current->name;

        xmlNodePtr requirementNode = current->xmlChildrenNode;
        while(requirementNode != NULL)
        {
            if(nameMatches(requirementNode, "spatial-requirement"))
            {
                LOG_DEBUG_S << "Parse spatial requirement";
                requirement.spatial = parseSpatialRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed spatial requirement: " << requirement.spatial.toString();

            } else if(nameMatches(requirementNode, "temporal-requirement"))
            {
                LOG_DEBUG_S << "Parse temporal requirement";
                requirement.temporal = parseTemporalRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed temporal requirement: " << requirement.temporal.toString();
            } else if(nameMatches(requirementNode, "service-requirement"))
            {
                LOG_DEBUG_S << "Parse functional requirement";
                requirement.functional = parseServiceRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed service requirement: " << requirement.functional.toString();
            } else if(nameMatches(requirementNode, "resource-requirement"))
            {
                LOG_DEBUG_S << "Parse resource requirement";
                requirement.resources = parseResourceRequirement(doc, requirementNode);
                LOG_DEBUG_S << "Parsed resource requirement: " << requirement.resources.toString();
            }
            requirementNode = requirementNode->next;
        }
    } else {
        throw std::invalid_argument("templ::io::MissionReader::parseRequirement: Unexpected tag: '" + std::string((const char*) current->name) + "' expected requirement");
    }
    return requirement;
}

std::vector<Requirement> MissionReader::parseRequirements(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_INFO_S << "Parsing: " << current->name;
    std::vector<Requirement> requirements;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "requirement"))
        {
            Requirement requirement = parseRequirement(doc, current);
            LOG_INFO_S << "Parsed requirement: " << requirement.toString();
            requirements.push_back(requirement);
        }
        current = current->next;
    }
    return requirements;
}

std::vector<TemporalConstraint> MissionReader::parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current)
{
    std::vector<TemporalConstraint> constraints;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(! (nameMatches(current,"text") || nameMatches(current, "comment")))
        {
            TemporalConstraint constraint;
            constraint.type = TemporalConstraint::getTemporalConstraintType( std::string((const char*) current->name) );
            constraint.lval = getProperty(current, "lval");
            constraint.rval = getProperty(current, "rval");

            LOG_DEBUG_S << "Parsed temporal constraint: " << constraint.toString();
            constraints.push_back(constraint);
        }

        current = current->next;
    }
    return constraints;
}

Constraints MissionReader::parseConstraints(xmlDocPtr doc, xmlNodePtr current)
{
    LOG_INFO_S << "Parsing: " << current->name;
    Constraints constraints;
    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "temporal-constraints"))
        {
            LOG_INFO_S << "Parsing: " << current->name;
            constraints.temporal = parseTemporalConstraints(doc, current);
        }
        current = current->next;
    }
    return constraints;
}

std::set<templ::symbols::Constant::Ptr> MissionReader::parseConstants(xmlDocPtr doc, xmlNodePtr current)
{
    std::set<templ::symbols::Constant::Ptr> constants;
    std::set<std::string> locations;

    current = current->xmlChildrenNode;
    while(current != NULL)
    {
        if(nameMatches(current, "location"))
        {
            std::string name = getSubNodeContent(doc, current, "id");

            if(locations.count(name))
            {
                throw std::invalid_argument("templ::io::MissionReader::parseConstants: location '" + name + "' defined"
                        " multiple times");
            }
            base::Point position;
            bool metricDefinition = false;
            try {
                position.x() = ::boost::lexical_cast<int32_t>( getSubNodeContent(doc, current, "x") );
                position.y() = ::boost::lexical_cast<int32_t>( getSubNodeContent(doc, current, "y") );
                position.z() = ::boost::lexical_cast<int32_t>( getSubNodeContent(doc, current, "z") );
                metricDefinition = true;
            } catch(const std::exception& e)
            {
                LOG_DEBUG_S << "Failed to extract metric location information: " << e.what();
            }

            if(!metricDefinition)
            {
                try {

                    double latitude = ::boost::lexical_cast<double>( getSubNodeContent(doc, current, "latitude") );
                    double longitude = ::boost::lexical_cast<double>( getSubNodeContent(doc, current, "longitude") );
                    std::string radius = getSubNodeContent(doc, current, "radius");
                    utils::CartographicMapping mapping(radius);
                    base::Point point(latitude, longitude, 0.0);
                    position = mapping.latitudeLongitudeToMetric(point);
                    LOG_INFO_S << "LatitudeLongitude information: " << point;
                } catch(const std::exception& e)
                {
                    throw std::runtime_error("templ::io::MissionReader::parseConstants: failed to extract location: " + std::string(e.what()));
                }
            }

            LOG_INFO_S << "Metric location information: " << position;
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

