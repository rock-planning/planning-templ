#ifndef TEMPL_DATA_PROPERTY_ASSIGNMENT_HPP
#define TEMPL_DATA_PROPERTY_ASSIGNMENT_HPP

#include <owlapi/model/IRI.hpp>
#include <organization_model/OrganizationModel.hpp>

namespace owlapi {
namespace model {
    class OWLOntologyTell;
} // end namespace model
} // end namespace owlapi

namespace templ {

/**
 * Class object to represent the particular assignment of a data property
 * This intends to allow override in the mission specification file, in order to
 * facilitate the handling of VRP based benchmarks.
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
    static void apply(organization_model::OrganizationModel::Ptr& om, const DataPropertyAssignment::List& assignments);

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
