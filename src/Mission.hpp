#ifndef TEMPL_MISSION_HPP
#define TEMPL_MISSION_HPP

#include <unordered_set>

#include <organization_model/OrganizationModel.hpp>
#include <organization_model/OrganizationModelAsk.hpp>
#include <organization_model/ModelPool.hpp>
#include <templ/solvers/Constraint.hpp>
#include <templ/ObjectVariable.hpp>
#include <templ/solvers/temporal/Interval.hpp>
#include <templ/solvers/temporal/PersistenceCondition.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/Role.hpp>

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
    solvers::temporal::QualitativeTemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;

protected:
    Mission(const std::string& name = "");

public:
    Mission(organization_model::OrganizationModel::Ptr om, const std::string& name = "");

    void setOrganizationModel(organization_model::OrganizationModel::Ptr organizationModel) { mpOrganizationModel = organizationModel; }

    /**
     * Set name for this mission
     */
    void setName(const std::string& name) { mName = name; }
    const std::string& getName() const { return mName; }

    void prepare();

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
     */
    void addResourceLocationCardinalityConstraint(
            const std::string& locationId,
            const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
            const solvers::temporal::point_algebra::TimePoint::Ptr& toTp,
            const owlapi::model::IRI& resourceModel,
            uint32_t cardinality = 1,
            owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type = owlapi::model::OWLCardinalityRestriction::MIN
            );

    /**
     */
    void addConstraint(const StateVariable& stateVariable,
            const ObjectVariable::Ptr& objectVariable,
            const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
            const solvers::temporal::point_algebra::TimePoint::Ptr& toTp);

    void addTemporalConstraint(const solvers::temporal::point_algebra::TimePoint::Ptr& t1,
            const solvers::temporal::point_algebra::TimePoint::Ptr& t2,
            solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type constraint);

    void addConstraint(const solvers::Constraint::Ptr& contraint);

    /**
     * Set the available resources
     */
    void setAvailableResources(const organization_model::ModelPool& modelPool);

    /**
     * Get the model pool of available resources
     */
    const organization_model::ModelPool& getAvailableResources() const { return mModelPool; }

    /**
     * Refresh information, e.g. after updating the resources
     */
    void refresh();

    ObjectVariable::Ptr getObjectVariable(const std::string& name, const std::string& type) const;
    ObjectVariable::Ptr getOrCreateObjectVariable(const std::string& name, const std::string& type) const;

    solvers::temporal::point_algebra::TimePoint::Ptr getTimePoint(const std::string& name) const;
    solvers::temporal::point_algebra::TimePoint::Ptr getOrCreateTimePoint(const std::string& name) const;

    const Role::List& getRoles() const { return mRoles; }

    /**
     * Get the list of involved models
     */
    const owlapi::model::IRIList& getModels() const { return mModels; }

    organization_model::OrganizationModel::Ptr getOrganizationModel() const { return mpOrganizationModel; }

    const owlapi::model::IRISet& getRequestedResources() const { return mRequestedResources; }

    const std::unordered_set<solvers::temporal::Interval>& getTimeIntervals() const { return mTimeIntervals; }

    const std::set<ObjectVariable::Ptr>& getObjectVariables() const { return mObjectVariables; }
    const owlapi::model::IRISet& getLocations() const { return mLocations; }

    solvers::temporal::QualitativeTemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const {
        return mpTemporalConstraintNetwork; }

    std::vector<solvers::temporal::PersistenceCondition::Ptr> getPersistenceConditions() const { return mPersistenceConditions; }

    std::string toString() const;

protected:
    std::vector<solvers::Constraint::Ptr> getConstraints() const { return mConstraints; }

    void validateAvailableResources() const;

private:
    organization_model::OrganizationModel::Ptr mpOrganizationModel;
    organization_model::OrganizationModelAsk mAsk;
    std::string mName;
    organization_model::ModelPool mModelPool;
    // Set of roles that exists within this mission
    Role::List mRoles;
    owlapi::model::IRIList mModels;

    std::vector<solvers::temporal::PersistenceCondition::Ptr> mPersistenceConditions;
    std::vector<solvers::Constraint::Ptr> mConstraints;

    // Structures to facilitate CSP definition
    owlapi::model::IRISet mRequestedResources;

    /// Since a sorted set required the less operator for sorting in is not a suitable
    /// suitable container for Interval, since overlapping intervals have to be considered
    std::unordered_set<solvers::temporal::Interval> mTimeIntervals;
    std::set<ObjectVariable::Ptr> mObjectVariables;
    owlapi::model::IRISet mLocations;
};

} // end namespace templ
#endif // TEMPL_MISSION_HPP
