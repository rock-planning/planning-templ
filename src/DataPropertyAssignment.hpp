#ifndef TEMPL_DATA_PROPERTY_ASSIGNMENT_HPP
#define TEMPL_DATA_PROPERTY_ASSIGNMENT_HPP

#include <owlapi/model/IRI.hpp>
#include <moreorg/OrganizationModel.hpp>

namespace owlapi {
namespace model {
    class OWLOntologyTell;
} // end namespace model
} // end namespace owlapi

namespace templ {

/**
 * Class object to represent the particular assignment of a data property
 * This intends to allow overrides in the mission specification file, in order to
 * facilitate the handling of VRP based benchmarks, e.g.
 \verbatim
    <?xml version="1.0" encoding="UTF-8"?>
    <mission>
      <name>Golden_1</name>
      <description/>
      <organization_model>http://www.rock-robotics.org/2017/11/vrp#</organization_model>
      <resources>
        <resource>
          <model>http://www.rock-robotics.org/2017/11/vrp#Commodity</model>
          <maxCardinality>20</maxCardinality>
        </resource>
        <resource>
          <model>http://www.rock-robotics.org/2017/11/vrp#Vehicle</model>
          <maxCardinality>10</maxCardinality>
        </resource>
      </resources>
      <overrides>
        <override>
          <subject>http://www.rock-robotics.org/2017/11/vrp#Vehicle</subject>
          <property>http://www.rock-robotics.org/2014/01/om-schema#transportCapacity</property>
          <value>550</value>
        </override>
      </overrides>
      <constants>
        <location>
          <id>depot0</id>
          <x>0</x>
          <y>0</y>
          <z>0</z>
        </location>
        ...
      <requirements>
        <requirement id="0">
          <spatial-requirement>
            <location>
              <id>l1</id>
            </location>
          </spatial-requirement>
          <temporal-requirement>
            <from>t1-0</from>
            <to>t1-1</to>
          </temporal-requirement>
          <resource-requirement>
            <resource>
              <model>http://www.rock-robotics.org/2017/11/vrp#Commodity</model>
              <minCardinality>1</minCardinality>
              <maxCardinality>1</maxCardinality>
            </resource>
          </resource-requirement>
        </requirement>
    </mission>
     \endverbatim
 * This reflect a OWL triple like statement
 */
class DataPropertyAssignment
{
public:
    typedef std::vector<DataPropertyAssignment> List;

    /**
     * Default constructor
     * \param subject The main subject
     * \param param The predicate or property to set
     * \param value The value to which the property will be set
     */
    DataPropertyAssignment(const owlapi::model::IRI& subject,
            const owlapi::model::IRI& predicate,
            double value);

    /**
     * Get the subject of this assignment
     * \return subject
     */
    const owlapi::model::IRI& getSubject() const { return mSubject; }

    /**
     * Get the predicate / property of this assignment
     * \return predicate
     */
    const owlapi::model::IRI& getPredicate() const { return mProperty; }

    /**
     * Get the value of this assignment
     * \return value
     */
    double getValue() const { return mValue; }

    /**
     * Apply a set of DataPropertyAssignment onto an existing
     * ontology/organization model
     */
    static void apply(moreorg::OrganizationModel::Ptr& om, const DataPropertyAssignment::List& assignments);

    /**
     * Apply this assignment to a ontology
     */
    void apply(owlapi::model::OWLOntologyAsk& ask, owlapi::model::OWLOntologyTell& tell) const;

    /**
     * Stringify this object
     */
    std::string toString(size_t indent = 0) const;

private:
    owlapi::model::IRI mSubject;
    owlapi::model::IRI mProperty;
    double mValue;
};
} // end namespace templ
#endif // TEMPL_DATA_PROPERTY_ASSIGNMENT_HPP
