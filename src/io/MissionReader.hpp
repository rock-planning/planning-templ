#ifndef TEMPL_IO_MISSION_READER_HPP
#define TEMPL_IO_MISSION_READER_HPP

#include <stdio.h>
#include <string>
#include <stdexcept>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "../Mission.hpp"

#include "MissionRequirements.hpp"
#include "../constraints/ModelConstraint.hpp"


namespace templ {
namespace io {

class MissionReader
{
public:
    /**
     * Construct a mission from given specification and url to an organization
     * model
     */
    static Mission fromFile(const std::string& url, const std::string& organizationModelUrl);

    /**
     * Construct a mission from given specification and organization model
     */
    static Mission fromFile(const std::string& url, const moreorg::OrganizationModel::Ptr& organizationModel = moreorg::OrganizationModel::Ptr());

private:
    static std::pair<owlapi::model::IRI, size_t> parseResource(xmlDocPtr doc, xmlNodePtr current);

    /**
     * Extrac the set of available resources
     */
    static moreorg::ModelPool parseResources(xmlDocPtr doc, xmlNodePtr current);

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
            const std::map<size_t, SpaceTime::SpaceIntervalTuple>& requirementIntervalMap,
            const std::map<std::string, SpaceTime::SpaceIntervalTuple>& locationIntervalMap);

    /**
     * An exammple for the model constraints
     *\verbatim
    <model-constraints>
        <all-distinct>
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </all-distinct>
        <min-distinct value="10">
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </min-distinct>
        <all-equal>
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </all-equal>
        <min-equal value="1">
            <model>http://www.rock-robotics.org/2014/01/om-schema#CoyoteIII</model>
            <requirements>0,1</requirements>
        </min-equal>
        <min-function value="10">
            <model>http://www.rock-robotics.org/2014/01/om-schema#TransportProvider</model>
            <requirements>0</requirements>
        </min-function>
        <min-property value="10">
            <model>http://www.rock-robotics.org/2014/01/om-schema#TransportProvider</model>
            <requirements>0</requirements>
            <property>http://www.rock-robotics.org/2014/01/om-schema#transportCapacity</property>
        </min-property>
        <!-- when general properties makes sense -->
        <max-property value="1000">
            <model>http://www.rock-robotics.org/2014/01/om-schema#TransportProvider</model>
            <requirements>0</requirements>
            <property>http://www.rock-robotics.org/2014/01/om-schema#mass</property>
        </max-property>
    </model-constraints>
     \endverbatim
     */
    static constraints::ModelConstraint::List parseModelConstraints(xmlDocPtr doc,
            xmlNodePtr current,
            const std::map<size_t, SpaceTime::SpaceIntervalTuple>& requirementIntervalMap,
            const std::map<std::string, SpaceTime::SpaceIntervalTuple>& locationIntervalMap);

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
        <moreorg>http://www.rock-robotics.org/2015/12/projects/TransTerrA</moreorg>
     </mission>
     \endverbatim
     */
    static moreorg::OrganizationModel::Ptr getOrganizationModel(const std::string& url);
};

} // end namespace io
} // end namespace templ

#endif // TEMPL_IO_MISSION_READER_HPP
