#ifndef TEMPL_PLANNING_STATE_HPP
#define TEMPL_PLANNING_STATE_HPP

#include <templ/solvers/csp/ModelDistribution.hpp>
#include <templ/solvers/csp/RoleDistribution.hpp>
//#include <templ/solvers/csp/RoleTimeline.hpp>
#include <templ/solvers/csp/Resolver.hpp>
#include <templ/TemporallyExpandedNetwork.hpp>

#include <graph_analysis/Vertex.hpp>

namespace templ {

class PlanningState : public graph_analysis::Vertex
{
public:
    enum SubStateFlag { 
        TEMPORAL = 0x01, 
        MODEL = 0x02, 
        ROLE = 0x04,
        ALL = 0xFF
    };

    typedef shared_ptr<PlanningState> Ptr;

    PlanningState(const Mission& mission, const std::string& label = "");

    PlanningState(const PlanningState& other);

    virtual ~PlanningState();

    const Mission& getMission() const { return mMission; }
    Mission& getMission() { return mMission; }

    void setMission(const Mission& mission);

    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> getTimePoints() { return mTimePoints; }
    std::vector<templ::symbols::constants::Location::Ptr> getLocations() { return mLocations; }



    solvers::temporal::TemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const {return mpTemporalConstraintNetwork; }
    void setTemporalConstraintNetwork(const solvers::temporal::TemporalConstraintNetwork::Ptr& tcn);

    void setModelDistribution(solvers::csp::ModelDistribution* m) { mpModelDistribution = m; }
    solvers::csp::ModelDistribution* getModelDistribution() const { return mpModelDistribution; }
    void setModelDistributionSearchEngine(Gecode::BAB<solvers::csp::ModelDistribution>* se) { mpModelDistributionSearchEngine = se; }
    Gecode::BAB<solvers::csp::ModelDistribution>* getModelDistributionSearchEngine() const { return mpModelDistributionSearchEngine; }

    const solvers::csp::ModelDistribution::Solution& getModelDistributionSolution() const { return mModelDistributionSolution; }
    void setModelDistributionSolution(const solvers::csp::ModelDistribution::Solution& solution) { mModelDistributionSolution = solution; }

    
    void setRoleDistribution(solvers::csp::RoleDistribution* r) { mpRoleDistribution = r; }
    solvers::csp::RoleDistribution* getRoleDistribution() const { return mpRoleDistribution; }
    void setRoleDistributionSearchEngine(Gecode::BAB<solvers::csp::RoleDistribution>* se) { mpRoleDistributionSearchEngine = se; }
    Gecode::BAB<solvers::csp::RoleDistribution>* getRoleDistributionSearchEngine() { return mpRoleDistributionSearchEngine; }

    const solvers::csp::RoleDistribution::Solution& getRoleDistributionSolution() const { return mRoleDistributionSolution; }
    void setRoleDistributionSolution(const solvers::csp::RoleDistribution::Solution& solution) { mRoleDistributionSolution = solution; }

    virtual std::string getClassName() { return "PlanningState"; }

    void cleanup(SubStateFlag flag = ALL);

    void addResolver(const solvers::csp::Resolver::Ptr& resolver) { mResolvers.push_back(resolver); }

    std::vector<solvers::csp::Resolver::Ptr>& getResolvers() { return mResolvers; }

    void removeResolver(const solvers::csp::Resolver::Ptr& resolver);

    void constrainMission(const symbols::constants::Location::Ptr& location,
            const solvers::temporal::Interval& interval,
            const owlapi::model::IRI& model,
            uint32_t cardinality = 1,
            owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type = owlapi::model::OWLCardinalityRestriction::MIN);

protected:
    virtual graph_analysis::Vertex* getClone() const { LOG_WARN_S << "CLONING BLUB"; return new PlanningState(*this); }

private:
    Mission mMission;
    solvers::temporal::TemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;

    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> mTimePoints;
    std::vector<templ::symbols::constants::Location::Ptr> mLocations;

    solvers::csp::ModelDistribution* mpModelDistribution;
    Gecode::BAB<solvers::csp::ModelDistribution>* mpModelDistributionSearchEngine;
    solvers::csp::ModelDistribution::Solution mModelDistributionSolution;

    solvers::csp::RoleDistribution* mpRoleDistribution;
    Gecode::BAB<solvers::csp::RoleDistribution>* mpRoleDistributionSearchEngine;
    solvers::csp::RoleDistribution::Solution mRoleDistributionSolution;

    // Current set of resolver that can be applied to fix the plan at this stage
    std::vector<solvers::csp::Resolver::Ptr> mResolvers;
};

} // end namespace templ
#endif // TEMPL_PLANNING_STATE_HPP
