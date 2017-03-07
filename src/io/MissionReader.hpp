#ifndef TEMPL_IO_MISSION_READER_HPP
#define TEMPL_IO_MISSION_READER_HPP

#include <stdio.h>
#include <string>
#include <stdexcept>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <templ/Mission.hpp>

#include "MissionRequirements.hpp"


namespace templ {
namespace io {

class MissionReader
{
public:
    static Mission fromFile(const std::string& url, const organization_model::OrganizationModel::Ptr& organizationModel);

private:
    static bool nameMatches(xmlNodePtr node, const std::string& name, bool useNamespace = false);

    static std::string resolveNamespacePrefix(xmlDocPtr doc, xmlNodePtr node, const std::string& prefix);

    static std::string getContent(xmlDocPtr doc, xmlNodePtr node, size_t count = 1);

    static std::string getProperty(xmlNodePtr node, const std::string& name);

    /**
     * Retrieve the content of a named subnode
     * \param doc xml document
     * \param node the current node
     * \param the name of the subnode
     * \throws std::invalid_argument if subnode of given name cannot be found
     */
    static std::string getSubNodeContent(xmlDocPtr doc, xmlNodePtr node, const std::string& name);

    static std::pair<owlapi::model::IRI, size_t> parseResource(xmlDocPtr doc, xmlNodePtr current);

    /**
     * Extrac the set of available resources
     */
    static organization_model::ModelPool parseResources(xmlDocPtr doc, xmlNodePtr current);

    static SpatialRequirement parseSpatialRequirement(xmlDocPtr doc, xmlNodePtr current);

    static TemporalRequirement parseTemporalRequirement(xmlDocPtr doc, xmlNodePtr current);

    /**
     *
     *\verbatim
        <resource-requirement>
            <resources>
                <resource>
                    <model>om:TransportProvider</model>
                    <minCardinality>1</minCardinality>
                    <attributes>
                        <numericAttribute name=om:payloadTransportCapacity>
                            <xsd:restriction>
                                <xsd:minInclusive>1</xsd:minInclusive>
                            </xsd:restriction>
                        </numericAttribute>
                    </attributes>
                </resource>
            </resources>
        </resource-requirement>
     \endverbatim
     */
    static std::vector<ResourceRequirement> parseResourceRequirements(xmlDocPtr doc, xmlNodePtr current);

    static ResourceReificationRequirement parseResourceReificationRequirement(xmlDocPtr doc, xmlNodePtr current);

    static NumericAttributeRequirements parseAttributes(xmlDocPtr doc, xmlNodePtr current);

    static SpatioTemporalRequirement parseRequirement(xmlDocPtr doc, xmlNodePtr current);

    static std::vector<SpatioTemporalRequirement> parseRequirements(xmlDocPtr doc, xmlNodePtr current);

    static std::vector<TemporalConstraint> parseTemporalConstraints(xmlDocPtr doc, xmlNodePtr current);

    static Constraints parseConstraints(xmlDocPtr doc, xmlNodePtr current);

    static std::set<templ::symbols::Constant::Ptr> parseConstants(xmlDocPtr doc, xmlNodePtr current);
};

} // end namespace io
} // end namespace templ

#endif // TEMPL_IO_MISSION_READER_HPP
