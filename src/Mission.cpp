#include "Mission.hpp"

namespace templ {

Mission::Mission(organization_model::OrganizationModel::Ptr om)
    : mpOrganizationModel(om)
{}

void Mission::addConstraint(organization_model::Service service,
        ObjectVariable::Ptr location,
        solvers::temporal::point_algebra::TimePoint::Ptr from,
        solvers::temporal::point_algebra::TimePoint::Ptr to)
{
    // the combination of service and location represent a state variable
    // sloc
    // which needs to be translated into resource based state variables
    StateVariable sloc("Location",
            service.getModel().toString());

    using namespace solvers::temporal;
    PersistenceCondition::Ptr persistenceCondition = PersistenceCondition::getInstance(sloc,
            location, 
            from,
            to);

   mPersistenceConditions.push_back(persistenceCondition); 
}

void Mission::addConstraint(solvers::Constraint::Ptr constraint)
{
    mConstraints.push_back(constraint);
}

} // end namespace templ
