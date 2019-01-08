#ifndef TEMPL_BENCHMARK_MISSION_GENERATOR_HPP
#define TEMPL_BENCHMARK_MISSION_GENERATOR_HPP

#include "../Mission.hpp"
#include "VRPProblem.hpp"
#include "../SpaceTime.hpp"
#include <organization_model/OrganizationModel.hpp>

namespace templ {
namespace benchmark {

/**
 * General mission generator which allows to create
 * automatically random and/or solvable missions
 */
class MissionGenerator
{
public:
    MissionGenerator(const std::string& configurationFile = "");

    void loadConfiguration(const std::string& configurationFile);

    /**
     * Generate a feasible logistics network of a set of atomic agents
     * \return network a atomic agents transitions
     */
    SpaceTime::Network generateNetwork();

    /**
     * Sample from a generated network, to create a mission specification
     * where the network represents one of possible many feasible solutions
     * \return mission specification
     */
    Mission::Ptr sampleFromNetwork(const SpaceTime::Network& network);

    /**
     * Generate a mission
     */
    Mission::Ptr generate();

    static Mission::Ptr convert(const VRPProblem& vrp);

protected:
    static owlapi::model::IRI mVRPOntology;

    double mAreaX;
    double mAreaY;
    double mAreaZ;

    size_t mNumberOfLocations;
    size_t mNumberOfTimepoints;
    organization_model::ModelPool mMinPool;
    organization_model::ModelPool mMaxPool;
    organization_model::ModelPool mAgentPool;
    size_t mMinAgentCount;
    size_t mMaxAgentCount;

    double mRatioImmobileMobile;
    double mSamplingDensity;

    bool mUseFunctionalities;
    // or maybe mix the mission
    double mRatioFunctionaltiesAgents;

    organization_model::OrganizationModel::Ptr mpOrganizationModel;
};

} // end namespace benchmark
} // end namespace templ
#endif // TEMPL_BENCHMARK_MISSION_GENERATOR_HPP
