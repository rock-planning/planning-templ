#include "Mission.hpp"
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>

namespace templ {

namespace pa = solvers::temporal::point_algebra;

Role::Role()
    : mName("unknown")
    , mModel()
{}

Role::Role(const std::string& name, const owlapi::model::IRI& model)
    : mName(name)
    , mModel(model)
{}

std::string Role::toString() const
{
    std::stringstream ss;
    ss << "Role: " << mName << " (" << mModel.toString() << ")";
    return ss.str();
}

std::string Role::toString(const Role::List& roles)
{
    std::stringstream ss;
    Role::List::const_iterator cit = roles.begin();
    ss << "Roles: " << std::endl;
    for(; cit != roles.end(); ++cit)
    {
        ss << "    - " << cit->toString() << std::endl;
    }
    return ss.str();
}

Mission::Mission(organization_model::OrganizationModel::Ptr om)
    : mpOrganizationModel(om)
    , mpTemporalConstraintNetwork(new solvers::temporal::QualitativeTemporalConstraintNetwork())
{}

void Mission::setResources(const organization_model::ModelPool& modelPool)
{
    mModelPool = modelPool;
    refresh();
}

void Mission::refresh()
{
    mRoles.clear();
    mModels.clear();

    organization_model::ModelPool::const_iterator cit = mModelPool.begin();
    for(;cit != mModelPool.end(); ++cit)
    {
        const owlapi::model::IRI& model = cit->first;
        size_t count = cit->second;

        // Update models
        mModels.push_back(model);

        // Update roles
        for(size_t i = 0; i < count; ++i)
        {
            std::stringstream ss;
            ss << model.getFragment() << "_" << i;
            Role role(ss.str(), model);
            mRoles.push_back(role);
        }
    }
}

void Mission::prepare()
{
    using namespace solvers::temporal;
    std::vector<PersistenceCondition::Ptr>::const_iterator cit =  mPersistenceConditions.begin();
    for(;cit != mPersistenceConditions.end(); ++cit)
    {
        PersistenceCondition::Ptr pc = *cit;
        Interval interval(pc->getFromTimePoint(), pc->getToTimePoint(), point_algebra::TimePointComparator(mpTemporalConstraintNetwork));
        mTimeIntervals.insert(interval);
    }
}

void Mission::addConstraint(organization_model::Service service,
        ObjectVariable::Ptr location,
        solvers::temporal::point_algebra::TimePoint::Ptr from,
        solvers::temporal::point_algebra::TimePoint::Ptr to)
{
    // the combination of service and location represent a state variable
    // sloc
    // which needs to be translated into resource based state variables
    StateVariable sloc("service-location",
            service.getModel().toString());

    mInvolvedServices.insert(service.getModel());
    mObjectVariables.insert(location);

    using namespace solvers::temporal;
    PersistenceCondition::Ptr persistenceCondition = PersistenceCondition::getInstance(sloc,
            location,
            from,
            to);

    mpTemporalConstraintNetwork->addConstraint(from, to, pa::QualitativeTimePointConstraint::LessOrEqual);


    mPersistenceConditions.push_back(persistenceCondition);
}

void Mission::addTemporalConstraint(pa::TimePoint::Ptr t1, pa::TimePoint::Ptr t2, pa::QualitativeTimePointConstraint::Type constraint)
{
    mpTemporalConstraintNetwork->addConstraint(t1, t2, constraint);
}

void Mission::addConstraint(solvers::Constraint::Ptr constraint)
{
    mConstraints.push_back(constraint);
}

} // end namespace templ
