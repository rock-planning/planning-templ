#ifndef TEMPL_MISSION_HPP
#define TEMPL_MISSION_HPP

#include <organization_model/OrganizationModel.hpp>
#include <organization_model/ModelPool.hpp>
#include <templ/solvers/Constraint.hpp>
#include <templ/ObjectVariable.hpp>
#include <templ/solvers/temporal/Interval.hpp>
#include <templ/solvers/temporal/PersistenceCondition.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <unordered_set>

namespace templ {

typedef uint32_t Duration;
typedef std::string ServiceLocation;

class MissionPlanner;

namespace io {
    class MissionReader;
}

/**
 * Role within the mission, representing an individual
 * system and the corresponding model
 */
class Role
{
    std::string mName;
    owlapi::model::IRI mModel;
public:
    typedef std::vector<Role> List;

    Role();
    Role(const std::string& name, const owlapi::model::IRI& model);

    std::string toString() const;
    static std::string toString(const std::vector<Role>& roles);
};

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

    void addConstraint(organization_model::Service service,
            ObjectVariable::Ptr location,
            solvers::temporal::point_algebra::TimePoint::Ptr fromTp,
            solvers::temporal::point_algebra::TimePoint::Ptr toTp);

    void addTemporalConstraint(solvers::temporal::point_algebra::TimePoint::Ptr t1,
            solvers::temporal::point_algebra::TimePoint::Ptr t2,
            solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type constraint);

    void addConstraint(solvers::Constraint::Ptr contraint);

    void setResources(const organization_model::ModelPool& modelPool);

    const organization_model::ModelPool& getResources() const { return mModelPool; }

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

    const owlapi::model::IRISet& getInvolvedServices() const { return mInvolvedServices; }
    const std::unordered_set<solvers::temporal::Interval>& getTimeIntervals() const { return mTimeIntervals; }
    const std::set<ObjectVariable::Ptr>& getObjectVariables() const { return mObjectVariables; }

    solvers::temporal::QualitativeTemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const {
        return mpTemporalConstraintNetwork; }

    std::vector<solvers::temporal::PersistenceCondition::Ptr> getPersistenceConditions() const { return mPersistenceConditions; }

    std::string toString() const;

protected:
    std::vector<solvers::Constraint::Ptr> getConstraints() const { return mConstraints; }

private:
    organization_model::OrganizationModel::Ptr mpOrganizationModel;
    std::string mName;
    organization_model::ModelPool mModelPool;
    // Set of roles that exists within this mission
    Role::List mRoles;
    owlapi::model::IRIList mModels;

    std::vector<solvers::temporal::PersistenceCondition::Ptr> mPersistenceConditions;
    std::vector<solvers::Constraint::Ptr> mConstraints;

    // Structures to facilitate CSP definition
    owlapi::model::IRISet mInvolvedServices;
    /// Since a sorted set required the less operator for sorting in is not a suitable
    /// suitable container for Interval, since overlapping intervals have to be considered
    std::unordered_set<solvers::temporal::Interval> mTimeIntervals;
    std::set<ObjectVariable::Ptr> mObjectVariables;
};

} // end namespace templ
#endif // TEMPL_MISSION_HPP
