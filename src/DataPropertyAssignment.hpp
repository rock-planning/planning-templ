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

class DataPropertyAssignment
{
public:
    typedef std::vector<DataPropertyAssignment> List;

    DataPropertyAssignment(const owlapi::model::IRI& subject,
            const owlapi::model::IRI& predicate,
            double value);

    const owlapi::model::IRI& getSubject() const { return mSubject; }
    const owlapi::model::IRI& getPredicate() const { return mProperty; }
    double getValue() const { return mValue; }

    static void apply(organization_model::OrganizationModel::Ptr& om, const DataPropertyAssignment::List& assignments);

    void apply(owlapi::model::OWLOntologyAsk& ask, owlapi::model::OWLOntologyTell& tell) const;

    std::string toString(size_t indent = 0) const;

private:
    owlapi::model::IRI mSubject;
    owlapi::model::IRI mProperty;
    double mValue;
};
} // end namespace templ
#endif // TEMPL_DATA_PROPERTY_ASSIGNMENT_HPP
