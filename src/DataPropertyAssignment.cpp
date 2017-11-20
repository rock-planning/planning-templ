#include "DataPropertyAssignment.hpp"
#include <sstream>
#include <owlapi/model/OWLOntologyAsk.hpp>
#include <owlapi/model/OWLOntologyTell.hpp>
#include <owlapi/model/OWLLiteral.hpp>

using namespace organization_model;
using namespace owlapi::model;

namespace templ {

DataPropertyAssignment::DataPropertyAssignment(const owlapi::model::IRI& subject,
            const owlapi::model::IRI& property,
            double value)
    : mSubject(subject)
    , mProperty(property)
    , mValue(value)
{
}

void DataPropertyAssignment::apply(organization_model::OrganizationModel::Ptr& om, const DataPropertyAssignment::List& assignments)
{
    OWLOntologyAsk ask(om->ontology());
    OWLOntologyTell tell(om->ontology());

    for(const DataPropertyAssignment& assignment : assignments)
    {
        assignment.apply(ask, tell);
    }
}

void DataPropertyAssignment::apply(owlapi::model::OWLOntologyAsk& ask, owlapi::model::OWLOntologyTell& tell) const
{
    // Update the property value for all instance of a given type
    std::stringstream ss;
    ss << mValue;
    IRIList instances = ask.allInstancesOf(mSubject, false);
    OWLLiteral::Ptr value = OWLLiteral::create(ss.str(), owlapi::vocabulary::XSD::resolve("double"));
    for(const IRI& instance : instances)
    {
        tell.valueOf(instance, mProperty, value);
    }
}

std::string DataPropertyAssignment::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "subject: " << mSubject << ", property: " << mProperty << ", value: " << mValue << std::endl;
    return ss.str();
}

} // end namespace templ
