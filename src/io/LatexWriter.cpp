#include "LatexWriter.hpp"
#include "../solvers/FluentTimeResource.hpp"
#include <boost/algorithm/string.hpp>

using namespace organization_model;

namespace templ {
namespace io {

std::string LatexWriter::toLatex(const Mission::Ptr& mission)
{
    std::stringstream ss;
    std::vector<solvers::FluentTimeResource> ftrs = Mission::getResourceRequirements(mission);

    ss << "\\begin{align*}" << std::endl;

    // General available
    ss << "    \\widehat{GA}\n&= \\big \\{";
    ModelPool available = mission->getAvailableResources();
    for(const ModelPool::value_type& v : available)
    {
        const owlapi::model::IRI& model = v.first;
        size_t cardinality = v.second;

        ss << "(" << escape( model.getFragment() ) << "," << cardinality << ")";
    }
    ss << "\\big \\} \\\\" << std::endl;


    // Requirements
    ss << "    STR" << std::endl;
    bool init = false;
    for(const solvers::FluentTimeResource& ftr : ftrs)
    {
        if(!init)
        {
            ss << "     &= \\big \\{";
            init = true;
        } else {
            ss << "     & ";
        }

        ss << toLatex(ftr) << ", \\\\" << std::endl;
    }
    ss << "    & \\big \\}\\\\" << std::endl;

    // Constraints
    ss << "    \\mathcal{X}" << std::endl;
    ss << "    &= \\big \\{ ";

    std::string row;
    for(const Constraint::Ptr& c : mission->getConstraints())
    {
        using namespace solvers::temporal::point_algebra;
        if(c->getCategory() == Constraint::TEMPORAL_QUALITATIVE)
        {
            QualitativeTimePointConstraint::Ptr qtpc =
                dynamic_pointer_cast<QualitativeTimePointConstraint>(c);

            std::string constraintString = qtpc->getLVal()->getLabel() +
                QualitativeTimePointConstraint::TypeSymbol[ qtpc->getType() ]
                + qtpc->getRVal()->getLabel();
            row = wrap(ss, row, constraintString);
        }
    }
    ss << row << " \\\\" << std::endl;
    ss << "    & \\big \\} \\\\" << std::endl;

    // Organization Model
    ss << "    \\mathcal{OM}" << std::endl;
    ss << "    &= \\big \\{ \\\\"  << std::endl;
    ss << "    & \\big \\} \\\\" << std::endl;

    // Timepoints
    ss << "    T\n";
    ss << "     &= \\big \\{ ";
    row = "";
    init = false;
    for(const solvers::temporal::point_algebra::TimePoint::Ptr& tp :
            mission->getTimepoints())
    {
        row = wrap(ss, row, tp->getLabel());
    }
    ss << row <<  " \\\\" << std::endl;
    ss << "    & \\big \\}\\\\" << std::endl;

    // Locations
    ss << "    L" << std::endl;
    init = false;
    for(const symbols::constants::Location::Ptr& location :
            mission->getLocations(true))
    {
        if(!init)
        {
            ss << "     &= \\big \\{ ";
            init = true;
        } else {
            ss << "     & ";
        }
        ss << toLatex(location) << ", \\\\" << std::endl;
    }
    ss << "    & \\big \\} \\\\" << std::endl;

    ss << "\\end{align*}" << std::endl;
    return ss.str();
}

std::string LatexWriter::toLatex(const symbols::constants::Location::Ptr& location)
{
    std::stringstream ss;
    base::Point point = location->getPosition();
    std::string locationLabel = location->getInstanceName();
    locationLabel = suffixNumber(locationLabel);

    ss << escape( locationLabel ) << "= (" << point.x() << ","
        << point.y() << "," << point.z() << ") ";
    return ss.str();
}

std::string LatexWriter::toLatex(const solvers::FluentTimeResource& ftr)
{
    owlapi::model::IRIList functionalities;
    std::stringstream ssAgents;

    organization_model::OrganizationModelAsk ask = ftr.getOrganizationModelAsk();

    ModelPool pool = ftr.getMinCardinalities();
    for(ModelPool::value_type& v : pool)
    {
        const owlapi::model::IRI& model = v.first;
        size_t count  = v.second;
        if(ask.ontology().isSubClassOf(model, vocabulary::OM::Functionality()))
        {
            functionalities.push_back(model);
        } else {
            ssAgents << "(" << escape( model.getFragment() ) << "," << count << ")";
        }
    }

    std::stringstream ss;
    ss << "(";
    Resource::Set resources = ftr.getRequiredResources();
    if(resources.empty())
    {
        ss << "\\emptyset";
    } else {
        ss << "\\{";
        Resource::Set::const_iterator cit = resources.begin();
        for(; cit != resources.end(); ++cit)
        {
            const Resource& r = *cit;
            ss << r.getModel().getFragment();
            if( std::distance(cit,resources.end()) != 1)
            {
                ss << ",";
            }
        }
        ss << "\\}";
    }
    ss << ",";
    ss << "\\{" << ssAgents.str() << "\\})";
    ss << "@(" << suffixNumber( ftr.getLocation()->getInstanceName() ) << ",";
    ss << "[" << suffixNumber( ftr.getInterval().getFrom()->getLabel() )
        << "," << suffixNumber( ftr.getInterval().getTo()->getLabel() )
        << "])";

   return ss.str();
}

std::string LatexWriter::suffixNumber(const std::string& label)
{
    std::string newlabel = label;
    std::vector<char> numbers{'0','1','2','3','4','5','6','7','8','9'};
    std::string::iterator it = std::find_first_of(newlabel.begin(), newlabel.end(), numbers.begin(), numbers.end());
    std::string suffixStart("_{");
    if(it != newlabel.end())
    {
        newlabel.insert(it,suffixStart.begin(), suffixStart.end());
        newlabel += "}";
    }
    return newlabel;
}

std::string LatexWriter::escape(const std::string& label,
        const std::string& env)
{
    std::string escapedLabel = label;
    boost::replace_all(escapedLabel, "_", + "\\_");
    if(!env.empty())
    {
        return env + "{" + escapedLabel + "}";
    } else {
        return escapedLabel;
    }
}

std::string LatexWriter::wrap(std::ostream& os,
        const std::string& currentRow,
        const std::string& newLabel,
        const std::string& suffixNoWrap,
        const std::string& suffixWrap,
        size_t lineWidth)
{
    std::string row;
    // Check row length
    if(currentRow.size() + newLabel.size() > lineWidth)
    {
        // write the remaining label
        os << currentRow <<  suffixWrap;
        row = "    & " + newLabel + suffixNoWrap;
    } else {
        row = currentRow + newLabel + suffixNoWrap;
    }
    return row;
}

} // end namespace io
} // end namespace templ
