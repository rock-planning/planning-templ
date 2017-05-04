#ifndef TEMPL_MISSION_HPP
#define TEMPL_MISSION_HPP

#include <unordered_set>

#include <organization_model/OrganizationModel.hpp>
#include <organization_model/OrganizationModelAsk.hpp>
#include <organization_model/ModelPool.hpp>
#include <templ/solvers/Constraint.hpp>
#include <templ/solvers/temporal/Interval.hpp>
#include <templ/solvers/temporal/PersistenceCondition.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/Role.hpp>
#include <templ/symbols/ObjectVariable.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/solvers/GQReasoner.hpp>
#include <templ/utils/Logger.hpp>

namespace templ {

typedef uint32_t Duration;
typedef std::string ServiceLocation;

class MissionPlanner;

namespace io {
    class MissionReader;
}

class Mission
{
    friend class MissionPlanner;
    friend class io::MissionReader;
    friend class PlanningState;

    solvers::temporal::TemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;
    graph_analysis::BaseGraph::Ptr mpRelations;

public:
    typedef shared_ptr<Mission> Ptr;

    Mission(organization_model::OrganizationModel::Ptr om, const std::string& name = "");

    Mission(const Mission& other);

    void setOrganizationModel(organization_model::OrganizationModel::Ptr organizationModel);

    /**
     * Set name for this mission
     */
    void setName(const std::string& name) { mName = name; }
    const std::string& getName() const { return mName; }

    graph_analysis::BaseGraph::Ptr getRelations() const { return mpRelations; }

    /**
     * Set the known timeintervals from the set of persistence conditions
     */
    void prepareTimeIntervals();

     /**
      * Add general constraint
      * \param constraint
      */
    void addConstraint(const solvers::Constraint::Ptr& constraint);

    /**
     * \param stateVariable
     * \param objectVariable
     * \param fromTp
     * \param toTp
     * \return the constraint that had been added
     */
    solvers::temporal::TemporalAssertion::Ptr addTemporalAssertion(const symbols::StateVariable& stateVariable,
            const symbols::ObjectVariable::Ptr& objectVariable,
            const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
            const solvers::temporal::point_algebra::TimePoint::Ptr& toTp);

    // Methods to add of specialized constraints
    //
    /**
     * Add a constraint that bind a resource (Service or Actor) to a specific
     * location with a given cardinality
     *
     * \param locationId
     * \param fromTp
     * \param toTp
     * \param resourceModel,
     * \param cardinality
     * \param type
     * \return the constraint that has been added
     */
    solvers::temporal::TemporalAssertion::Ptr addResourceLocationCardinalityConstraint(
            const symbols::constants::Location::Ptr& location,
            const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
            const solvers::temporal::point_algebra::TimePoint::Ptr& toTp,
            const owlapi::model::IRI& resourceModel,
            uint32_t cardinality = 1,
            owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type = owlapi::model::OWLCardinalityRestriction::MIN
            );

    solvers::temporal::TemporalAssertion::Ptr addResourceLocationNumericAttributeConstraint(
            const symbols::constants::Location::Ptr& location,
            const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
            const solvers::temporal::point_algebra::TimePoint::Ptr& toTp,
            const owlapi::model::IRI& resourceModel,
            const owlapi::model::IRI& attribute,
            int32_t minInclusive,
            int32_t maxInclusive
            );



    solvers::Constraint::Ptr addTemporalConstraint(const solvers::temporal::point_algebra::TimePoint::Ptr& t1,
            const solvers::temporal::point_algebra::TimePoint::Ptr& t2,
            solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type constraint);

    //void addReificationConstraint(const

    /**
     * Set the available resources
     */
    void setAvailableResources(const organization_model::ModelPool& modelPool);

    /**
     * Get the model pool of available resources
     */
    const organization_model::ModelPool& getAvailableResources() const { return mModelPool; }

    /**
     * Refresh internal datastructures, e.g. after updating the list of
     * available resources
     */
    void refresh();


    const Role::List& getRoles() const { return mRoles; }

    /**
     * Get the list of involved models
     */
    const owlapi::model::IRIList& getModels() const { return mModels; }

    organization_model::OrganizationModel::Ptr getOrganizationModel() const { return mpOrganizationModel; }

    const owlapi::model::IRIList& getRequestedResources() const { return mRequestedResources; }

    /**
     * Get all involved time intervals
     */
    const std::vector<solvers::temporal::Interval>& getTimeIntervals() const { return mTimeIntervals; }

    const std::set<symbols::ObjectVariable::Ptr>& getObjectVariables() const { return mObjectVariables; }
    symbols::ObjectVariable::Ptr getObjectVariable(const std::string& name, symbols::ObjectVariable::Type type) const;
    symbols::ObjectVariable::Ptr getOrCreateObjectVariable(const std::string& name, symbols::ObjectVariable::Type type) const;

    const std::set<symbols::Constant::Ptr>& getConstants() const { return mConstants; }
    const symbols::Constant::Ptr& getConstant(const std::string& id, symbols::Constant::Type type = symbols::Constant::UNKNOWN);

    solvers::temporal::TemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const {
        return mpTemporalConstraintNetwork; }

    std::vector<solvers::temporal::PersistenceCondition::Ptr> getPersistenceConditions() const { return mPersistenceConditions; }

    std::string toString() const;

    /**
     * Get special sets of constants
     * \param excludeUnused Set to true to exclude unused constants, i.e. which
     * are not required for or used within the definition of the mission
     * \return list of location constants
     */
    std::vector<symbols::constants::Location::Ptr> getLocations(bool excludeUnused = true) const;

    /**
     * Get the timepoints ordered by the associated temporal constraint network
     * \return timepoints ordered (earlier entries correspond to earlier times)
     */
    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> getTimepoints() const;

    void save(const std::string& filename) const { throw std::runtime_error("Mission::save: not implemented"); }

    /**
     * Get access to the OrganizationModelAsk object to query information from
     * the associated organization model
     */
    const organization_model::OrganizationModelAsk& getOrganizationModelAsk() const { return mAsk; }

    /**
     * Set the logger that is associated with this mission object
     */
    void setLogger(const Logger::Ptr& logger) { mpLogger = logger; }

    /**
     * Get the logger that is associated with this mission
     * \return logger
     */
    Logger::Ptr getLogger() const { return mpLogger; }

    /**
     * The path to the scenario file that was used to load this mission file
     * (if loaded) -- will be empty if was not loaded from a file
     */
    const std::string& getScenarioFile() const { return mScenarioFile; }

    std::vector<solvers::Constraint::Ptr> getConstraints() const { return mConstraints; }

    /**
     * Check if mission is ready to be forwarded to planner
     * \throws std::runtime_error if mission is not ready to be used for planning
     */
    void validateForPlanning() const;

    /**
     * Add relation
     */
    graph_analysis::Edge::Ptr addRelation(graph_analysis::Vertex::Ptr source,
            const std::string& label,
            graph_analysis::Vertex::Ptr target);

protected:
    void requireConstant(const symbols::Constant::Ptr& constant);

    void incrementConstantUse(const symbols::Constant::Ptr& constant);

    /**
     * Add a constant
     * \param constant Ptr object of this constant
     * \throw std::invalid_argument if the constant is already registered (but
     * has a different pointer)
     */
    void addConstant(const symbols::Constant::Ptr& constant);

    /**
     * Check if there are some resources available at all
     */
    void validateAvailableResources() const;

    /**
     * Set the path to the scenario file that was used to load this mission file
     */
    void setScenarioFile(const std::string& filename) { mScenarioFile = filename; }

private:
    organization_model::OrganizationModel::Ptr mpOrganizationModel;
    organization_model::OrganizationModelAsk mAsk;
    std::string mName;
    // The set of available resources
    organization_model::ModelPool mModelPool;
    // Set of roles that exists within this mission
    Role::List mRoles;
    // The list of involved models
    owlapi::model::IRIList mModels;

    std::vector<solvers::temporal::PersistenceCondition::Ptr> mPersistenceConditions;
    std::vector<solvers::Constraint::Ptr> mConstraints;

    // Structures to facilitate CSP definition
    owlapi::model::IRIList mRequestedResources;

    /// Since a sorted set required the less operator for sorting in is not a suitable
    /// suitable container for Interval, since overlapping intervals have to be considered
    std::vector<solvers::temporal::Interval> mTimeIntervals;

    std::set<symbols::ObjectVariable::Ptr> mObjectVariables;
    std::set<symbols::Constant::Ptr> mConstants;
    mutable std::map<symbols::Constant::Ptr, uint32_t> mConstantsUse;

    std::string mScenarioFile;
    Logger::Ptr mpLogger;
};

} // end namespace templ
#endif // TEMPL_MISSION_HPP
