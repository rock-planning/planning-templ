#ifndef TEMPL_BENCHMARK_MISSION_GENERATOR_HPP
#define TEMPL_BENCHMARK_MISSION_GENERATOR_HPP

#include "../Mission.hpp"
#include "VRPProblem.hpp"

namespace templ {
namespace benchmark {

class MissionGenerator
{
public:
    static Mission::Ptr convert(const VRPProblem& vrp);

protected:
    static owlapi::model::IRI mVRPOntology;

};

} // end namespace benchmark
} // end namespace templ
#endif // TEMPL_BENCHMARK_MISSION_GENERATOR_HPP
