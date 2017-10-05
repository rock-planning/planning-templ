#include "../../../Mission.hpp"

namespace templ {
namespace solvers {
namespace csp {
namespace utils {

class Converter
{
public:
static std::set< std::vector<uint32_t> > toCSP(const Mission::Ptr& mission, const organization_model::ModelPool::Set& combinations);

static std::vector<uint32_t> toCSP(const Mission::Ptr& mission, const organization_model::ModelPool& combination);

static uint32_t systemModelToCSP(const Mission::Ptr& mission, const owlapi::model::IRI& model);

};

} // end namespace utils
} // end namespace csp
} // end namespace solvers
} // end namespace templ
