#include "Flaw.hpp"

using namespace graph_analysis::algorithms;

namespace templ {
namespace solvers {
namespace transshipment {

std::string Flaw::toString() const
{
    std::stringstream ss;
    switch(violation.getType())
    {
        case ConstraintViolation::MinFlow:
                ss << "Minflow violation in timeline: enforce distiction on timeline for: " << affectedRole.toString()
                    << " between " << std::endl
                    << previousFtr.toString() << " and " << ftr.toString() << std::endl
                    << previousFtr.getInterval().toString() << std::endl
                    << ftr.getInterval().toString() << std::endl
                    ;
                    //<< " timeline: " << std::endl
                    //<< roleTimeline.toString()
                    //<< std::endl;
                break;
        case ConstraintViolation::TransFlow:
                ss << "Transflow violation in timeline: enforce distiction on timeline for: " << affectedRole.toString()
                    << " between " << std::endl
                    << ftr.toString() << " and " << subsequentFtr.toString() << std::endl
                    << ftr.getInterval().toString() << std::endl
                    << subsequentFtr.getInterval().toString() << std::endl
                    ;
                    //<< " timeline: " << std::endl
                    //<< roleTimeline.toString()
                    //<< std::endl;
                break;

    }
    return ss.str();
}


} // end namespace transshipment
} // end namespace solvers
} // end namespace templ
