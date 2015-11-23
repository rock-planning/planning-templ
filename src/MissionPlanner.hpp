#ifndef TEMPL_MISSION_PLANNER_HPP
#define TEMPL_MISSION_PLANNER_HPP

#include <vector>
#include <stack>
#include <templ/solvers/temporal/Chronicle.hpp>
#include <templ/Mission.hpp>
#include <organization_model/OrganizationModelAsk.hpp>
#include <templ/solvers/csp/ModelDistribution.hpp>
#include <templ/solvers/csp/RoleDistribution.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>
#include <templ/solvers/csp/Resolver.hpp>
#include <templ/LocationTimepointTuple.hpp>
#include <graph_analysis/algorithms/MultiCommodityMinCostFlow.hpp>

/**
 * \mainpage TemPl -- A resource planner for missions with reconfigurable multi-robot systems
 *
 * \section planning Planning method
 *
 * The planning consists of multiple steps
 * 1. planning the model distribution
 * 2. planning the role distribution
 * 3. generation of the timeline (plan) per role and analysis of the cost for
 * the timeline and the overall plan
 *
 * \section spec Mission specification
 * The easiest way to use the planner (in its current status) is to define a mission using the XML
 * specification.
 *
 * A mission specification the model information to
 * be consistent with a given organization model (and the related ontology to be
 * loaded)
 *
 * The resource section defines the available resources for a given mission
 * planner, e.g., in term of robot models that are available.
 *
 * The requirements section define what kind of services are required when and where.
 *
 * The constraints section allows to define qualitative relationships between
 * timepoints.
 *\verbatim
<mission>
<name>Transterra</name>
<resources>
    <resource>
        <model>http://www.rock-robotics.org/2014/01/om-schema#Sherpa</model>
        <maxCardinality>1</maxCardinality>
    </resource>
    <resource>
        <model>http://www.rock-robotics.org/2014/01/om-schema#CREX</model>
        <maxCardinality>0</maxCardinality>
    </resource>
    <resource>
        <model>http://www.rock-robotics.org/2014/01/om-schema#Payload</model>
        <maxCardinality>0</maxCardinality>
    </resource>
</resources>
<requirements>
    <requirement>
        <spatial-requirement>
            <!-- where it is required -->
            <location>
                <id>l0</id>
            </location>
        </spatial-requirement>

        <!-- when it is required / mixing qualitative and quantitative
        information -->
        <temporal-requirement type="persistence-condition | event" >
            <from>t0</from>
            <to>t1</to>
        </temporal-requirement>

        <!-- what is required at this very position -->
        <service-requirement>
            <service>http://www.rock-robotics.org/2014/01/om-schema#LocationImageProvider</service>
            <service>http://www.rock-robotics.org/2014/01/om-schema#EmiPowerProvider</service>
        </service-requirement>
        <resource-requirement>
            <resource>
                <model>http://www.rock-robotics.org/2014/01/om-schema#CREX</model>
                <minCardinality>1</minCardinality>
            </resource>
        </resource-requirement>
    </requirement>
    <requirement>
        <spatial-requirement>
            <!-- where it is required -->
            <location>
                <id>l1</id>
            </location>
        </spatial-requirement>
        <temporal-requirement>
            <from>t2</from>
            <to>t3</to>
        </temporal-requirement>

        <!-- what is required at this very position -->
        <service-requirement>
            <service>http://www.rock-robotics.org/2014/01/om-schema#LocationImageProvider</service>
            <service>http://www.rock-robotics.org/2014/01/om-schema#EmiPowerProvider</service>
        </service-requirement>
        <resource-requirement>
            <resource>
                <model>http://www.rock-robotics.org/2014/01/om-schema#CREX</model>
                <minCardinality>1</minCardinality>
            </resource>
        </resource-requirement>
    </requirement>
</requirements>
<constraints>
    <temporal-constraints>
        <!--     <greaterThan lval="t1" rval="t2" />
        <lessThan lval="t0" rval="t3" />-->
    </temporal-constraints>
</constraints>

</mission>
 \endverbatim
 *
 *
 * In order
 *\verbatim
    using namespace templ;
    ...
    Mission mission = io::MissionReader::fromFile(missionFilename);

    using namespace organization_model;
    OrganizationModel::Ptr organizationModel = OrganizationModel::getInstance(organizationModelFilename);
    mission.setOrganizationModel(organizationModel);

    std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
    if(!solutions.empty())
    {
        std::cout << "Solutions for model distribution found: " << solutions << std::endl;
    }

    // Check role distribution after model distribution
    std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
    if(!roleSolutions.empty())
    {
        std::cout << "Solutions for role distribution: " << roleSolutions << std::endl;
    }
 \endverbatim
 *
 *
 */
namespace templ {

typedef std::vector<Mission> CandidateMissions;

class MissionPlanner
{
    friend class templ::solvers::csp::Resolver;
    friend class templ::solvers::csp::RoleAddDistinction;
    friend class templ::solvers::csp::FunctionalityRequest;

public:
    MissionPlanner(const Mission& mission, const organization_model::OrganizationModel::Ptr& organizationModel);

    ~MissionPlanner();

    void execute(uint32_t maxIterations);

    bool nextModelAssignment();
    bool nextRoleAssignment();

    void computeRoleTimelines();
    void computeTemporallyExpandedLocationNetwork();
    std::vector<graph_analysis::algorithms::ConstraintViolation> computeMinCostFlow();

    // Save the intermediate results
    void save(const std::string& markerLabel = "", const std::string& dir = "/tmp") const;

protected:
    Mission mCurrentMission;
    organization_model::OrganizationModel::Ptr mOrganizationModel;
    organization_model::OrganizationModelAsk mOrganizationModelAsk;
    owlapi::model::OWLOntologyAsk mOntologyAsk;

    solvers::csp::ModelDistribution* mModelDistribution;
    solvers::csp::ModelDistribution::Solution mModelDistributionSolution;
    Gecode::BAB<solvers::csp::ModelDistribution>* mModelDistributionSearchEngine;

    solvers::csp::RoleDistribution* mRoleDistribution;
    solvers::csp::RoleDistribution::Solution mRoleDistributionSolution;
    Gecode::BAB<solvers::csp::RoleDistribution>* mRoleDistributionSearchEngine;
    std::map<Role, solvers::csp::RoleTimeline> mRoleTimelines;
    uint32_t mCommodities;

    typedef std::pair< templ::symbols::constants::Location::Ptr, templ::solvers::temporal::point_algebra::TimePoint::Ptr> LocationTimePointPair;
    std::map< LocationTimePointPair, LocationTimepointTuple::Ptr > mTupleMap;


    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> mTimepoints;
    solvers::temporal::point_algebra::TimePointComparator mTimePointComparator;
    std::vector<templ::symbols::constants::Location::Ptr> mLocations;

    graph_analysis::BaseGraph::Ptr mSpaceTimeGraph;
    graph_analysis::BaseGraph::Ptr mFlowGraph;

    // Current set of resolver that can be applied to fix the plan at this stage
    std::vector<solvers::csp::Resolver::Ptr> mResolvers;

};

} // end namespace templ
#endif // TEMPL_MISSION_PLANNER_HPP
