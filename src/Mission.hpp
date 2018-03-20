#ifndef TEMPL_MISSION_HPP
#define TEMPL_MISSION_HPP

#include <unordered_set>

#include <organization_model/OrganizationModel.hpp>
#include <organization_model/OrganizationModelAsk.hpp>
#include <organization_model/ModelPool.hpp>

#include "Constraint.hpp"
#include "solvers/temporal/Interval.hpp"
#include "solvers/temporal/PersistenceCondition.hpp"
#include "solvers/temporal/point_algebra/TimePoint.hpp"
#include "solvers/temporal/QualitativeTemporalConstraintNetwork.hpp"
#include "Role.hpp"
#include "symbols/ObjectVariable.hpp"
#include "symbols/constants/Location.hpp"
#include "utils/Logger.hpp"
#include "DataPropertyAssignment.hpp"

namespace templ {

typedef uint32_t Duration;
typedef std::string ServiceLocation;

class MissionPlanner;

namespace io {
    class MissionReader;
    class SpatioTemporalRequirement;
}

namespace benchmark {
    class MissionGenerator;
}

namespace solvers {
    struct FluentTimeResource;
} // end namespace solvers

/**
 * \class Mission
 * \brief Description of a mission involving a reconfigurable multi-robot system
 *
 */
class Mission
{
    friend class MissionPlanner;
    friend class io::MissionReader;
    friend class PlanningState;
    friend class benchmark::MissionGenerator;

    solvers::temporal::TemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;
    graph_analysis::BaseGraph::Ptr mpRelations;

public:
    typedef shared_ptr<Mission> Ptr;

    /**
     * Default constructor to allow usage in maps
     */
    Mission();

    Mission(const organization_model::OrganizationModel::Ptr& om, const std::string& name = "");

    Mission(const Mission& other);

    /**
     * Set name for this mission
     * \param name Name of the mission
     */
    void setName(const std::string& name) { mName = name; }

    /**
     * Get the name for this mission
     * \return name of mission
     */
    const std::string& getName() const { return mName; }

    /**
     * Set a description for this mission
     * \param description
     */
    void setDescription(const std::string& description) { mDescription = description; }

    /**
     * Get description
     * \return description
     */
    const std::string& getDescription() const { return mDescription; }

    /**
     * Set the organization model in use -- property values assigned through the
     * organization model, can be overriden using the data property assignment
     */
    void setOrganizationModel(const organization_model::OrganizationModel::Ptr& organizationModel);

    /**
     * Get the organization model
     * \return organization_model
     */
    organization_model::OrganizationModel::Ptr getOrganizationModel() const { return mpOrganizationModel; }

    /**
     * Get access to the OrganizationModelAsk object to query information from
     * the associated organization model
     */
    const organization_model::OrganizationModelAsk& getOrganizationModelAsk() const { return mOrganizationModelAsk; }

    /**
     * Set a list of DataPropertyAssignments
     * \see addDataPropertyAssignment
     */
    void setDataPropertyAssignments(const DataPropertyAssignment::List& a) { mDataPropertyAssignments = a; }

    /**
     * Set data property assignments to allow overrides, e.g., to facilitate handling of VRP related
     * problem instances in the context of benchmarking
     *
     * A data property assignment for a agent model, allows to set data
     * properties, after the organization model has been read
     */
    void addDataPropertyAssignment(const DataPropertyAssignment& da) { mDataPropertyAssignments.push_back(da); }

    /**
     * Get the data property assignments
     */
    const DataPropertyAssignment::List& getDataPropertyAssignments() const { return mDataPropertyAssignments; }

    /**
     * Sets the available resources in terms of available agent models, and
     * triggers a refresh of the mission This also reinitializes the
     * organization model query, which accounts for the functional saturation
     * bound
     * \param modelPool List of resources and cardinalities
     */
    void setAvailableResources(const organization_model::ModelPool& modelPool);

    /**
     * Get the model pool of available resources in terms of available agent
     * models
     */
    const organization_model::ModelPool& getAvailableResources() const { return mModelPool; }

    /**
     * Get all locations as defined along with the mission specification
     * \param excludeUnused Set to true to exclude unused constants, i.e. which
     * are not required for or used within the definition of the mission
     * \return list of location constants
     */
    std::vector<symbols::constants::Location::Ptr> getLocations(bool excludeUnused = true) const;

    /**
     * Get location by name
     * \param name of location
     * \return timepoint
     */
    symbols::constants::Location::Ptr getLocation(const std::string& name) const;

    /**
     * Get the timepoints unordered, i.e. all timepoints known to the mission
     * \return timepoints unordered
     */
    solvers::temporal::point_algebra::TimePoint::PtrList getUnorderedTimepoints() const
    { return mTimePoints; }


    /**
     * Get the timepoints ordered by the associated temporal constraint network
     * \param sorted sort timepoints if possible
     * \return timepoints ordered (earlier entries correspond to earlier times)
     */
    solvers::temporal::point_algebra::TimePoint::PtrList getOrderedTimepoints() const;

    /**
     * Redirects to getOrderedTimepoints
     * \deprecated Prefer using getUnorderedTimepoints or getOrderedTimepoints
     * instead according to the requirements
     */
    solvers::temporal::point_algebra::TimePoint::PtrList getTimepoints() const { return getOrderedTimepoints(); }

    /**
     * Get timepoint by name
     * \param name of timepoint
     * \return timepoint
     */
    solvers::temporal::point_algebra::TimePoint::Ptr getTimepoint(const std::string& name) const;

    /**
     * Get the special transfer-location
     */
    symbols::constants::Location::Ptr getTransferLocation() const { return mpTransferLocation; }

    /**
     * Retrieve the set of relations, i.e. mapping the existing persistence
     * conditions to the set of requirements
     *
     */
    graph_analysis::BaseGraph::Ptr getRelations() const { return mpRelations; }

    /**
     * Get all roles that can possibly be associated with this mission,
     * according and derives from the available resources
     * \return list
     * \see getAvailableResources
     */
    const Role::List& getRoles() const { return mRoles; }

    /**
     * Get the list of involved models according to the available resources
     * \return list of models
     * \see getAvailableResources
     */
    const owlapi::model::IRIList& getModels() const { return mModels; }

    /**
     * The list of all functionalities that could be requested (e.g. for mission
     * repair) and the list of available agent types that are relevant in this
     * mission (provided through the mission specification)
     * \see addResourceLocationCardinality
     */
    const owlapi::model::IRIList& getRequestedResources() const { return mRequestedResources; }

    /**
     * Get all time intervals of the mission specification, available after
     * prepareTimeIntervals has been called
     * \see prepareTimeIntervals
     */
    const std::vector<solvers::temporal::Interval>& getTimeIntervals() const { return mTimeIntervals; }

    /**
     * Get the timeinterval range from the timepoint given by from, to timepoint
     * given by to
     * \return Interval
     * \throw std::invalid_argument if Interval could not be found among the
     * existing
     */
    const solvers::temporal::Interval& getTimeInterval(const std::string& from, const std::string& to) const;

    /**
     * Get the set of object variables associated with the mission
     */
    const std::set<symbols::ObjectVariable::Ptr>& getObjectVariables() const { return mObjectVariables; }

    /**
     * Get the object variable given by name and type
     * \return object variable
     * \throws std::invalid_argument when variable cannot be found
     */
    symbols::ObjectVariable::Ptr getObjectVariable(const std::string& name, symbols::ObjectVariable::Type type) const;

    /**
     * Get the object variable given by name and types if it exists, otherwise
     * it will be created
     * \return object variable
     */
    symbols::ObjectVariable::Ptr getOrCreateObjectVariable(const std::string& name, symbols::ObjectVariable::Type type) const;

    /**
     * Get the set of known constants, e.g., Locations
     */
    const std::set<symbols::Constant::Ptr>& getConstants() const { return mConstants; }

    /**
     * Get constant by name
     * \throws std::invalid_argument when the constant cannot be found
     */
    const symbols::Constant::Ptr& getConstant(const std::string& id, symbols::Constant::Type type = symbols::Constant::UNKNOWN) const;

    /**
     * Get the temporal constraint network associated with the mission
     * \return temporal constraint network
     */
    solvers::temporal::TemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const {
        return mpTemporalConstraintNetwork; }

    /**
     * Get the qualitative temporal constraint network associated with the mission
     * \return temporal constraint network
     */
    solvers::temporal::QualitativeTemporalConstraintNetwork::Ptr getQualitativeTemporalConstraintNetwork() const {
        using namespace solvers::temporal;
        QualitativeTemporalConstraintNetwork::Ptr qtcn = dynamic_pointer_cast<solvers::temporal::QualitativeTemporalConstraintNetwork>(mpTemporalConstraintNetwork);
        if(!qtcn)
        {
            throw std::invalid_argument("templ::Mission::getQualitativeTemporalConstraintNetwork: could not cast temporal constraint network to qualitiative temporal constraint network");
        }
        return qtcn;
    }


    /**
     * Get the list of persistence conditions defined for this mission
     * \return persistence conditions
     */
    std::vector<solvers::temporal::PersistenceCondition::Ptr> getPersistenceConditions() const { return mPersistenceConditions; }

    /**
     * Set the logger that is associated with this mission object
     * \param logger Logger instance
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

    /**
     * Get the list of constraints
     * \return constraint of this mission
     */
    const std::vector<Constraint::Ptr>& getConstraints() const { return mConstraints; }

    /**
     * Add relation from source to target
     * \param source
     * \param label
     * \param target
     */
    graph_analysis::Edge::Ptr addRelation(const graph_analysis::Vertex::Ptr& source,
            const std::string& label,
            const graph_analysis::Vertex::Ptr& target);

    /**
     * Get the requirement that is associated witha particular requirement id
     * \param requirement id
     * \return requirement
     * \throw std::invalid_argument if no instance with given id could be found
     */
    shared_ptr<io::SpatioTemporalRequirement> getRequirementById(uint32_t id) const;

    /**
     * Find the fluent time resource which is related to the spatio temporal
     * requirement as defined in the mission specification
     * \param requirement the requirement as parsed out of the mission
     * specification
     * \param ftrs the already processed and generated list of
     * FluentTimeResource
     * \return FluentTimeResource related to the requirement
     * \throw std::invalid_argument if no related instance could be found
     */
    solvers::FluentTimeResource findRelated(const shared_ptr<io::SpatioTemporalRequirement>& requirement,
            const std::vector<solvers::FluentTimeResource>& ftrs) const;

    /**
     * Find the fluent time resource which is related to the spatio temporal
     * requirement with the given id as defined in the mission specification
     * \param id of the requirement in the mission specification
     * \param ftrs the already processed and generated list of
     * FluentTimeResource
     * \return FluentTimeResource related to the requirement
     * \throw std::invalid_argument if no related instance could be found
     */
    solvers::FluentTimeResource findRelatedById(uint32_t id,
            const std::vector<solvers::FluentTimeResource>& ftrs) const;

    /// UTILITY FUNCTIONS

    /**
     * Apply any overrides to the organization model
     * This functionality intends to facilitate the reading of VRP based files
     */
    void applyOrganizationModelOverrides();

    /**
     * Prepare the known timeintervals from the set of persistence conditions
     */
    void prepareTimeIntervals();

     /**
      * Add general constraint
      * \param constraint
      * TODO: this is current limited to constraint edges -- better would be a
      * hyperedge
      */
    void addConstraint(const Constraint::Ptr& constraint);

    /**
     * Check if mission has a particular constraint
     * \return True, if the mission has this constraint, false otherwise
     */
    bool hasConstraint(const Constraint::Ptr& constraint) const;

    /**
     * Adds a temporal assertion, i.e. the assertion of a state variable to a
     * particular value/objectVariable
     *
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
            size_t cardinality = 1,
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

    /**
     * Refresh internal datastructures, e.g. after updating the list of
     * available resources
     */
    void refresh();


    /**
     * Convert to string representation
     * \return string
     */
    std::string toString() const;

    /**
     * Save mission representation to file
     */
    void save(const std::string& filename) const;

    /**
     * Check if mission is ready to be forwarded to planner, i.e.,
     * checks that time intervals are available, the temporal constraint
     * network is consistent, and checks that there are available resources
     * \throws std::runtime_error if mission is not ready to be used for planning
     */
    void validateForPlanning() const;

    /**
     * Update the max cardinalites according the the available resources of a
     * mission
     */
    void updateMaxCardinalities(solvers::FluentTimeResource& ftr) const;

    /**
     * Return the list of resource requirements
     * Requirements are sorted based on the from value
     * \param mission Pointer to mission -- this pointer will be associated
     * with all resource requirements (thus we do not simply use a reference
     * here)
     * \return list of FluentTimeResource
     */
    static std::vector<solvers::FluentTimeResource> getResourceRequirements(const Mission::Ptr& mission);

    /**
     * Get a fluent time resource from a Persistence
     * condition (using LocationCardinality)
     * \param p PersistenceCondition to convert
     * \param mission pointer to a mission
     */
    static solvers::FluentTimeResource fromLocationCardinality(const solvers::temporal::PersistenceCondition::Ptr& p, const Mission::Ptr& mission);

    /**
     * Save the mission and the organization model files
     * \param path Path to a folder to save the mission and organization model
     * files
     */
    void saveInputData(const std::string& path) const;

    /**
     * Enable the use of a transfer location
     */
    void enableTransferLocation() { requireConstant(mpTransferLocation); }

protected:
    /**
     * Require a constant to be declared
     */
    void requireConstant(const symbols::Constant::Ptr& constant);

    /**
     * Increment an internal use counter for a constant
     */
    uint32_t incrementConstantUse(const symbols::Constant::Ptr& constant);

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
    organization_model::OrganizationModelAsk mOrganizationModelAsk;
    /// Name of the mission
    std::string mName;
    /// Description of the mission
    std::string mDescription;
    // The set of available resources
    organization_model::ModelPool mModelPool;
    // Set of available roles that exists within this mission based on the
    // possible instantiations of the model pool
    Role::List mRoles;
    // The list of involved models
    owlapi::model::IRIList mModels;

    /// The persistance conditions defining the mission
    std::vector<solvers::temporal::PersistenceCondition::Ptr> mPersistenceConditions;
    /// Any mission constraints such as: model constraints and temporal
    /// constraints
    Constraint::PtrList mConstraints;

    /// The list of resources that are defining the domain for
    /// the CSP
    owlapi::model::IRIList mRequestedResources;

    /// Since a sorted set required the less operator for sorting in is not a suitable
    /// suitable container for Interval, since overlapping intervals have to be considered
    std::vector<solvers::temporal::Interval> mTimeIntervals;
    // Timepoints associated with the time intervals
    solvers::temporal::point_algebra::TimePoint::PtrList mTimePoints;

    std::set<symbols::ObjectVariable::Ptr> mObjectVariables;
    std::set<symbols::Constant::Ptr> mConstants;
    mutable std::map<symbols::Constant::Ptr, uint32_t> mConstantsUse;

    /// The specially added transfer location
    symbols::constants::Location::Ptr mpTransferLocation;

    std::string mScenarioFile;
    Logger::Ptr mpLogger;

    /// Particular overrides to set/override the properties of the agents
    /// This allows to facilitate the managment of VRP benchmark files
    DataPropertyAssignment::List mDataPropertyAssignments;
};

} // end namespace templ
#endif // TEMPL_MISSION_HPP
