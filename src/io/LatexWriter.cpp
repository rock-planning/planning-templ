#include "LatexWriter.hpp"
#include "../solvers/FluentTimeResource.hpp"

using namespace organization_model;

namespace templ {
namespace io {

std::string LatexWriter::toLatex(const Mission::Ptr& mission)
{
    std::stringstream ss;
    std::vector<solvers::FluentTimeResource> ftrs = Mission::getResourceRequirements(mission);

    ss << "\\begin{align*}" << std::endl;

    // General available
    ss << "    \\widehat{GA}  &= \\{";
    ModelPool available = mission->getAvailableResources();
    for(const ModelPool::value_type& v : available)
    {
        const owlapi::model::IRI& model = v.first;
        size_t cardinality = v.second;

        ss << "(" << model.getFragment() << "," << cardinality << ")";
    }
    ss << "\\} \\\\" << std::endl;


    // Requirements
    ss << "    STR ";
    bool init = false;
    for(const solvers::FluentTimeResource& ftr : ftrs)
    {
        if(!init)
        {
            ss << "      &= \\{";
            init = true;
        } else {
            ss << "     & ";
        }

        ss << toLatex(ftr) << ", \\\\" << std::endl;
    }
    ss << "    & \\}\\\\" << std::endl;

    ss << "\\end{align*}" << std::endl;

    for(const symbols::constants::Location::Ptr& location : mission->getLocations(true))
    {
        base::Point point = location->getPosition();
        std::string locationLabel = location->getInstanceName();
        locationLabel = suffixNumber(locationLabel);

        ss << locationLabel << "= (" << point.x() << ","
            << point.y() << ", " << point.z() << "), ";
    }
    return ss.str();
}

std::string LatexWriter::toLatex(const solvers::FluentTimeResource& ftr)
{
    owlapi::model::IRIList functionalities;
    std::stringstream ssAgents;

    OrganizationModelAsk ask = ftr.getMission()->getOrganizationModelAsk();

    ModelPool pool = ftr.getMinCardinalities();
    for(ModelPool::value_type& v : pool)
    {
        const owlapi::model::IRI& model = v.first;
        size_t count  = v.second;
        if(ask.ontology().isSubClassOf(model, vocabulary::OM::Functionality()))
        {
            functionalities.push_back(model);
        } else {
            ssAgents << "(" << model.getFragment() << "," << count << ")";
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

} // end namespace io
} // end namespace templ
