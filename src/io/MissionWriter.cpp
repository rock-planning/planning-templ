#include "MissionWriter.hpp"
#include <qxcfg/utils/XMLUtils.hpp>
#include "../solvers/FluentTimeResource.hpp"
#include "../solvers/temporal/point_algebra/QualitativeTimePointConstraint.hpp"
#include "../solvers/temporal/IntervalConstraint.hpp"
#include "../io/MissionRequirements.hpp"
#include "../constraints/ModelConstraint.hpp"

using namespace qxcfg::utils;
using namespace templ::solvers;
namespace pa = templ::solvers::temporal::point_algebra;

namespace templ {
namespace io {

void MissionWriter::write(const std::string& path, const Mission& mission, const std::string& encoding)
{
    xmlTextWriterPtr writer;
    xmlDocPtr doc;

    doc = xmlNewDoc(BAD_CAST XML_DEFAULT_VERSION);
    if(doc == NULL)
    {
        throw std::runtime_error("templ::io::MissionWriter::write: error creating"
                " xml document");
    }

    // Create a new XmlWriter for uri with no compression
    writer = xmlNewTextWriterDoc(&doc, 0);
    if(writer == NULL)
    {
        throw std::runtime_error("templ::io::MissionWriter::write: failed to open file '"
                + path + "'");
    }

    // Start the document with the xml default for the version and given encoding
    int rc = xmlTextWriterStartDocument(writer, NULL, encoding.c_str(), NULL);
    if(rc < 0)
    {
        throw std::runtime_error("templ::io::MissionWriter::write: failed to"
                " start document:  '" + path + "'");
    }

    Mission::Ptr mission_p(new Mission(mission));
    mission_p->prepareTimeIntervals();

    XMLUtils::startElement(writer, "mission");

    XMLUtils::startElement(writer, "name");
    XMLUtils::writeString(writer, mission.getName());
    XMLUtils::endElement(writer); // end name

    XMLUtils::startElement(writer, "description");
    XMLUtils::writeString(writer, mission.getDescription());
    XMLUtils::endElement(writer); // end description

    XMLUtils::startElement(writer, "organization_model");
    XMLUtils::writeString(writer, mission.getOrganizationModel()->ontology()->getIRI().toString());
    XMLUtils::endElement(writer); // end organization_model

    XMLUtils::startElement(writer, "resources");
    for(const organization_model::ModelPool::value_type& r : mission.getAvailableResources())
    {
        XMLUtils::startElement(writer, "resource");
        XMLUtils::startElement(writer, "model");
        XMLUtils::writeString(writer, r.first.toString());
        XMLUtils::endElement(writer); // end model
        XMLUtils::startElement(writer, "maxCardinality");
        std::stringstream ss;
        ss << r.second;
        XMLUtils::writeString(writer, ss.str());
        XMLUtils::endElement(writer); // end maxCardinality
        XMLUtils::endElement(writer); // end resource
    }
    XMLUtils::endElement(writer); // end resources;

    XMLUtils::startElement(writer, "overrides");
    for(const DataPropertyAssignment& assignment : mission.getDataPropertyAssignments())
    {
        XMLUtils::startElement(writer, "override");
            XMLUtils::startElement(writer, "subject");
            XMLUtils::writeString(writer, assignment.getSubject().toString());
            XMLUtils::endElement(writer); // end subject

            XMLUtils::startElement(writer, "property");
            XMLUtils::writeString(writer, assignment.getPredicate().toString());
            XMLUtils::endElement(writer); // end property

            XMLUtils::startElement(writer, "value");
            std::stringstream ss;
            ss << assignment.getValue();
            XMLUtils::writeString(writer, ss.str());
            XMLUtils::endElement(writer); // end value
        XMLUtils::endElement(writer); // end override
    }
    XMLUtils::endElement(writer); // end overrides

    XMLUtils::startElement(writer, "constants");
    for(symbols::Constant::Ptr c : mission.getConstants())
    {
        symbols::constants::Location::Ptr location = dynamic_pointer_cast<symbols::constants::Location>(c);
        if(location->getInstanceName() == mission_p->getTransferLocation()->getInstanceName())
        {
            // skip internally generated type
            continue;
        }
        XMLUtils::startElement(writer, "location");
    // location(id,radius,latitude,longitude) || location(id,x,y,z)
        XMLUtils::startElement(writer, "id");
        XMLUtils::writeString(writer, location->getInstanceName());
        XMLUtils::endElement(writer); // end id
        XMLUtils::startElement(writer, "x");
        {
            std::stringstream ss;
            ss << location->getPosition().x();
            XMLUtils::writeString(writer, ss.str());
        }
        XMLUtils::endElement(writer);
        XMLUtils::startElement(writer, "y");
        {
            std::stringstream ss;
            ss << location->getPosition().y();
            XMLUtils::writeString(writer, ss.str());
        }
        XMLUtils::endElement(writer);
        XMLUtils::startElement(writer, "z");
        {
            std::stringstream ss;
            ss << location->getPosition().z();
            XMLUtils::writeString(writer, ss.str());
        }
        XMLUtils::endElement(writer);
        XMLUtils::endElement(writer); // end location

    }
    XMLUtils::endElement(writer); // end constants

    /// Record to allow proper referencing in constraints
    std::map<SpaceTime::SpaceIntervalTuple, size_t> intervalRequirementIdMap;
    XMLUtils::startElement(writer, "requirements");
    std::vector<FluentTimeResource> resources = Mission::getResourceRequirements(mission_p);
    int requirementId = 0;
    for(const FluentTimeResource& ftr : resources)
    {
        SpaceTime::SpaceIntervalTuple tuple(ftr.getLocation(),
                ftr.getInterval());
        intervalRequirementIdMap[tuple] = requirementId;

        XMLUtils::startElement(writer, "requirement");
        std::stringstream ss;
        ss << requirementId++;
        XMLUtils::writeAttribute(writer, "id", ss.str());

        XMLUtils::startElement(writer, "spatial-requirement");
        XMLUtils::startElement(writer, "location");
        XMLUtils::startElement(writer, "id");
        XMLUtils::writeString(writer, ftr.getLocation()->getInstanceName());
        XMLUtils::endElement(writer); // end location id
        XMLUtils::endElement(writer); // end location
        XMLUtils::endElement(writer); // end spatial requirement

        XMLUtils::startElement(writer, "temporal-requirement");
        XMLUtils::startElement(writer, "from");
        XMLUtils::writeString(writer, ftr.getInterval().getFrom()->getLabel());
        XMLUtils::endElement(writer); // end from
        XMLUtils::startElement(writer, "to");
        XMLUtils::writeString(writer, ftr.getInterval().getTo()->getLabel());
        XMLUtils::endElement(writer); // end to
        XMLUtils::endElement(writer); // end temporal requirement

        XMLUtils::startElement(writer, "resource-requirement");
        std::set<owlapi::model::IRI> models;
        {
            for(const std::pair<owlapi::model::IRI,size_t> p : ftr.getMinCardinalities())
            {
                models.insert(p.first);
            }
        }
        {
            for(const std::pair<owlapi::model::IRI,size_t> p : ftr.getMaxCardinalities())
            {
                models.insert(p.first);
            }
        }
        for(const owlapi::model::IRI& model : models)
        {
            size_t minCardinality = 0;
            size_t maxCardinality = std::numeric_limits<size_t>::max();

            // minCardinalities
            organization_model::ModelPool::const_iterator minIt =  ftr.getMinCardinalities().find(model);
            if(minIt != ftr.getMinCardinalities().end())
            {
                minCardinality = minIt->second;
            }
            organization_model::ModelPool::const_iterator maxIt =  ftr.getMaxCardinalities().find(model);
            if(maxIt != ftr.getMaxCardinalities().end())
            {
                if(maxIt->second != std::numeric_limits<size_t>::max())
                {
                    maxCardinality = maxIt->second;
                }
            }

            size_t modelBound = mission.getAvailableResources().getValue(model,
                    std::numeric_limits<size_t>::max());
            if(minCardinality > 0 || maxCardinality < modelBound)
            {
                XMLUtils::startElement(writer, "resource");
                XMLUtils::startElement(writer, "model");
                XMLUtils::writeString(writer, model.toString());
                XMLUtils::endElement(writer); // end model;

                std::stringstream sm;
                sm << minCardinality;
                XMLUtils::startElement(writer, "minCardinality");
                XMLUtils::writeString(writer, sm.str());
                XMLUtils::endElement(writer); // end minCardinality

                if(maxCardinality < modelBound)
                {
                    std::stringstream smax;
                    smax << maxIt->second;
                    XMLUtils::startElement(writer, "maxCardinality");
                    XMLUtils::writeString(writer, smax.str());
                    XMLUtils::endElement(writer); // end maxCardinality
                }

                XMLUtils::endElement(writer); // end resource
            }
        }
        XMLUtils::endElement(writer); // end resource-requirement
        XMLUtils::endElement(writer); // end requirement
    }
    // each do
    XMLUtils::endElement(writer); // end requirements


    std::vector<Constraint::Ptr> constraints = mission.getConstraints();
    XMLUtils::startElement(writer, "constraints");
    XMLUtils::startElement(writer, "temporal-constraints");
    for(const Constraint::Ptr& c : constraints)
    {
        if(c->getCategory() == Constraint::TEMPORAL_QUALITATIVE)
        {
            pa::QualitativeTimePointConstraint::Ptr qtpc = dynamic_pointer_cast<pa::QualitativeTimePointConstraint>(c);
            std::string tag = io::TemporalConstraint::toXML(qtpc->getType());
            XMLUtils::startElement(writer, tag);
            XMLUtils::writeAttribute(writer, "lval", qtpc->getSourceVariable()->getLabel());
            XMLUtils::writeAttribute(writer, "rval", qtpc->getTargetVariable()->getLabel());
            XMLUtils::endElement(writer);
        } else if(c->getCategory() == Constraint::TEMPORAL_QUANTITATIVE)
        {
            using namespace solvers::temporal;
            IntervalConstraint::Ptr ic = dynamic_pointer_cast<IntervalConstraint>(c);
            for(Bounds b : ic->getIntervals())
            {
                XMLUtils::startElement(writer, "duration");

                std::stringstream min;
                min << b.getLowerBound();
                XMLUtils::writeAttribute(writer, "min", min.str());

                if( b.getUpperBound() != std::numeric_limits<double>::max())
                {
                    std::stringstream max;
                    max << b.getUpperBound();
                    XMLUtils::writeAttribute(writer, "max", max.str());
                }

                XMLUtils::startElement(writer, "from");
                XMLUtils::writeString(writer, ic->getSourceTimePoint()->getLabel());
                XMLUtils::endElement(writer); // end from

                XMLUtils::startElement(writer, "to");
                XMLUtils::writeString(writer, ic->getTargetTimePoint()->getLabel());
                XMLUtils::endElement(writer); // end to

                XMLUtils::endElement(writer); // end duration
            }
        } else if(c->getCategory() == Constraint::UNKNOWN)
        {
            throw std::invalid_argument("templ::io::MissionWriter::write: cannot write constraint"
                    " of type 'UNKNOWN'");
        }
    }
    XMLUtils::endElement(writer); // end temporal-constraints
    XMLUtils::startElement(writer, "model-constraints");
    for(const Constraint::Ptr& c : constraints)
    {
        using namespace templ::constraints;
        if(c->getCategory() == Constraint::MODEL)
        {
            ModelConstraint::Ptr modelConstraint = dynamic_pointer_cast<ModelConstraint>(c);
            ModelConstraint::Type type = modelConstraint->getModelConstraintType();
            std::string tag = ModelConstraint::TypeTxt[type];

            XMLUtils::startElement(writer, tag);
            size_t value = modelConstraint->getValue();
            std::stringstream valueTxt;
            valueTxt << value;
            if(type != ModelConstraint::ALL_DISTINCT)
            {
                XMLUtils::writeAttribute(writer, "value", valueTxt.str());
            }
            XMLUtils::startElement(writer, "model");
            XMLUtils::writeString(writer, modelConstraint->getModel().toString());
            XMLUtils::endElement(writer); // end model
            XMLUtils::startElement(writer, "requirements");
            std::stringstream ss;
            for(const SpaceTime::SpaceIntervalTuple& t : modelConstraint->getSpaceIntervalTuples())
            {
                std::map<SpaceTime::SpaceIntervalTuple,size_t>::const_iterator cit =  intervalRequirementIdMap.find(t);
                if(cit == intervalRequirementIdMap.end())
                {
                    std::runtime_error("templ::io::MissionWriter::write: failed to identify the requirements that are associated with a model constraint");
                }
                ss << cit->second << ",";
            }
            if(type == ModelConstraint::MIN_PROPERTY || type == ModelConstraint::MAX_PROPERTY)
            {
                XMLUtils::startElement(writer, "property");
                XMLUtils::writeString(writer, modelConstraint->getProperty().toString());
                XMLUtils::endElement(writer);
            }
            std::string requirementIdsTxt = ss.str();
            XMLUtils::writeString(writer, requirementIdsTxt.substr(0, requirementIdsTxt.length()-1) );
            XMLUtils::endElement(writer); // end requirements
            XMLUtils::endElement(writer); // end 'tag'
        }
    }

    XMLUtils::endElement(writer); // end model-constraints


    XMLUtils::endElement(writer); // end constraints

    XMLUtils::endElement(writer);  // end mission

    xmlTextWriterEndDocument(writer);

    xmlFreeTextWriter(writer);
    xmlSaveFileEnc(path.c_str(), doc, encoding.c_str());
    xmlFreeDoc(doc);

    XMLUtils::lint(path);
}

} // end namespace io
} // end namespace templ
