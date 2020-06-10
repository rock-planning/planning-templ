#include "Converter.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace utils {

std::set< std::vector<uint32_t> > Converter::toCSP(const Mission::Ptr& mission, const moreorg::ModelPool::Set& combinations)
{
    std::set< std::vector<uint32_t> > csp_combinations;
    moreorg::ModelPool::Set::const_iterator cit = combinations.begin();
    for(; cit != combinations.end(); ++cit)
    {
        csp_combinations.insert( toCSP(mission, *cit) );
    }
    return csp_combinations;
}

std::vector<uint32_t> Converter::toCSP(const Mission::Ptr& mission, const moreorg::ModelPool& combination)
{
    // return index of model and count per model
    std::vector<uint32_t> csp_combination(mission->getAvailableResources().size(),0);

    moreorg::ModelPool::const_iterator cit = combination.begin();
    for(; cit != combination.end(); ++cit)
    {
        uint32_t index = systemModelToCSP(mission, cit->first);
        csp_combination.at(index) = cit->second;
    }
    return csp_combination;
}

uint32_t Converter::systemModelToCSP(const Mission::Ptr& mission, const owlapi::model::IRI& model)
{
    const owlapi::model::IRIList availableModels = mission->getModels();
    owlapi::model::IRIList::const_iterator cit = std::find(availableModels.begin(),
            availableModels.end(), model);

    if(cit == availableModels.end())
    {
        throw std::invalid_argument("templ::solvers::csp::utils::Converter::systemModelToCSP:"
                " unknown model '" + model.toString() );
    } else {
        return (cit - availableModels.begin());
    }

}

} // end namespace utils
} // end namespace csp
} // end namespace solvers
} // end namespace templ
