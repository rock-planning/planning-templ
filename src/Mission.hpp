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

class Mission
{
    friend class MissionPlanner;
    solvers::temporal::QualitativeTemporalConstraintNetwork::Ptr mpTemporalConstraintNetwork;

protected:
    Mission() {}

public:
    Mission(organization_model::OrganizationModel::Ptr om);

    void prepare();

    void addConstraint(organization_model::Service service,
            ObjectVariable::Ptr location,
            solvers::temporal::point_algebra::TimePoint::Ptr fromTp,
            solvers::temporal::point_algebra::TimePoint::Ptr toTp);

    void addTemporalConstraint(solvers::temporal::point_algebra::TimePoint::Ptr t1,
            solvers::temporal::point_algebra::TimePoint::Ptr t2,
            solvers::temporal::point_algebra::QualitativeTimePointConstraint::Type constraint);

    void addConstraint(solvers::Constraint::Ptr contraint);

    void setResources(const organization_model::ModelPool& modelPool) { mModelPool = modelPool; }

    const organization_model::ModelPool& getResources() const { return mModelPool; }

    organization_model::OrganizationModel::Ptr getOrganizationModel() const { return mpOrganizationModel; }

    const owlapi::model::IRISet& getInvolvedServices() const { return mInvolvedServices; }
    const std::unordered_set<solvers::temporal::Interval>& getTimeIntervals() const { return mTimeIntervals; }
    const std::set<ObjectVariable::Ptr>& getObjectVariables() const { return mObjectVariables; }

    solvers::temporal::QualitativeTemporalConstraintNetwork::Ptr getTemporalConstraintNetwork() const {
        return mpTemporalConstraintNetwork; }

    std::vector<solvers::temporal::PersistenceCondition::Ptr> getPersistenceConditions() const { return mPersistenceConditions; }

protected:
    std::vector<solvers::Constraint::Ptr> getConstraints() const { return mConstraints; }

private:
    organization_model::OrganizationModel::Ptr mpOrganizationModel;
    organization_model::ModelPool mModelPool;

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
