#include "../../../Mission.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace utils {

class Converter
{
public:
/**
 * Create a set containing the cardinality vectors that corresponding the set of model
 * pool combination
 * \return set of vectors
 */
static std::set< std::vector<uint32_t> > toCSP(const Mission::Ptr& mission, const moreorg::ModelPool::Set& combinations);

/**
 * Create a fully expanded vector sized the number of the missions available resources
 * The model pool updates the cardinality of relevant resources
 * \return a vector of resource cardinalities
 */
static std::vector<uint32_t> toCSP(const Mission::Ptr& mission, const moreorg::ModelPool& combination);

/**
 * Get the model index in the available models of the mission
 * \return index
 */
static uint32_t systemModelToCSP(const Mission::Ptr& mission, const owlapi::model::IRI& model);

};

} // end namespace utils
} // end namespace csp
} // end namespace solvers
} // end namespace templ
