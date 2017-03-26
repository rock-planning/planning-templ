#ifndef TEMPL_MISSION_PLANNER_HPP
#define TEMPL_MISSION_PLANNER_HPP

#include <vector>
#include <stack>
#include <graph_analysis/BipartiteGraph.hpp>
#include <graph_analysis/algorithms/DFS.hpp>
#include <organization_model/OrganizationModelAsk.hpp>

#include <templ/solvers/temporal/Chronicle.hpp>
#include <templ/Mission.hpp>
#include <templ/PlanningState.hpp>
#include <templ/solvers/csp/RoleTimeline.hpp>
#include <templ/SpaceTimeNetwork.hpp>
#include <templ/PlanSpaceVisitor.hpp>


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
namespace solvers {
    class GQReasoner;
}

typedef std::vector<Mission> CandidateMissions;

/**
 * \class MissionPlanner
 * \brief The mission planning core
 * \details Planning is divided into multiple steps
 * 1. model assignment
 * 2. role assignment
 * 3. solving the multi-commodity min cost flow
 */
class MissionPlanner
{
public:
    enum Type { BASIC, GRAPH_PLANNER };

    MissionPlanner(const Mission& mission);

    virtual ~MissionPlanner();

    /**
     * Execute planning with up to a given number of solutions
     * \return list of solution plans
     */
    virtual std::vector<Plan> execute(uint32_t maxIterations);

    /**
     * Compute the next temporal constraint network for this mission
     */
    graph_analysis::BaseGraph::Ptr nextTemporalConstraintNetwork();

protected:
    /**
     *
     * \brief Render a plan from the current solution
     * \throw std::runtime_error if an initial tuple could not be found
     * \throw std::invalid_argument if rendering of plan failed
     */
    Plan renderPlan(const Mission::Ptr& mission,
            SpaceTimeNetwork* spaceTimeNetwork,
            const std::map<Role, solvers::csp::RoleTimeline>& timelines,
            const std::string& markerLabel = "") const;

    Type getType() const { return mType; }

private:
    Mission mMission;
    solvers::GQReasoner* mpGQReasoner;
    Logger::Ptr mpLogger;
    Type mType;
};

} // end namespace templ
#endif // TEMPL_MISSION_PLANNER_HPP
