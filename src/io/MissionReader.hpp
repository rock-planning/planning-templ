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
    /**
     * Construct a mission from given specification and url to an organization
     * model
     */
    static Mission fromFile(const std::string& url, const std::string& organizationModelUrl = "");

    /**
     * Construct a mission from given specification and organization model
     */
    static Mission fromFile(const std::string& url, const organization_model::OrganizationModel::Ptr& organizationModel = organization_model::OrganizationModel::Ptr());

private:
    static std::pair<owlapi::model::IRI, size_t> parseResource(xmlDocPtr doc, xmlNodePtr current);

    /**
     * Extrac the set of available resources
     */
    static organization_model::ModelPool parseResources(xmlDocPtr doc, xmlNodePtr current);

    static SpatialRequirement parseSpatialRequirement(xmlDocPtr doc, xmlNodePtr current);

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

    /**
     * Override the data property value for all instances of a particular
     * subject
     *\verbatim
          <overrides>
            <override>
              <subject>http://www.rock-robotics.org/2014/01/om-schema#Agent</subject>
              <property>http://www.rock-robotics.org/2014/01/om-schema#transportCapacity</property>
              <value>550</value>
            </override>
          </overrides>
     \endverbatim
     */
    static DataPropertyAssignment::List parseOverrides(xmlDocPtr doc, xmlNodePtr current);

    static ResourceReificationRequirement parseResourceReificationRequirement(xmlDocPtr doc, xmlNodePtr current);

    static NumericAttributeRequirements parseAttributes(xmlDocPtr doc, xmlNodePtr current);

    static SpatioTemporalRequirement parseRequirement(xmlDocPtr doc, xmlNodePtr current);

    static std::vector<SpatioTemporalRequirement> parseRequirements(xmlDocPtr doc, xmlNodePtr current);

    /**
     * Parse the set of constants
     \verbatim
      <constants>
          <location>
              <id>lander</id>
              <radius>moon</radius>
              <latitude>-83.82009</latitude>
              <longitude>87.53932</longitude>
          </location>
      </constants>
     \endverbatim
     */
    static std::set<templ::symbols::Constant::Ptr> parseConstants(xmlDocPtr doc, xmlNodePtr current);


    static templ::io::Constraints parseConstraints(xmlDocPtr doc,
            xmlNodePtr current,
            const std::map<uint32_t, SpaceTime::SpaceIntervalTuple>& requirementIntervalMap);

    static MissionConstraint::List parsePlanningConstraints(xmlDocPtr doc,
            xmlNodePtr current,
            const std::map<uint32_t, SpaceTime::SpaceIntervalTuple>& requirementIntervalMap);

    /**
     * If the mission specification file contains a node organization model,
     * then the provided IRI will be taken to initialize load the organization
     * model.
     * If not provided, then other means (e.g. command line argument) might be
     * available. Still, this will be the preferred option to set the
     * organization model.
     *
     \verbatim
     <mission>
        ...
        <organization_model>http://www.rock-robotics.org/2015/12/projects/TransTerrA</organization_model>
     </mission>
     \endverbatim
     */
    static organization_model::OrganizationModel::Ptr getOrganizationModel(const std::string& url);
};

} // end namespace io
} // end namespace templ

#endif // TEMPL_IO_MISSION_READER_HPP
