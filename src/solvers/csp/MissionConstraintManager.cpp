#include "MissionConstraintManager.hpp"
#include "MissionConstraints.hpp"
#include "TransportNetwork.hpp"
#include "../../constraints/ModelConstraint.hpp"
#include <iostream>

namespace templ {
namespace solvers {
namespace csp {

void MissionConstraintManager::apply(const Constraint::Ptr& constraint,
        TransportNetwork& transportNetwork)
{
    switch(constraint->getCategory())
    {
        case Constraint::TEMPORAL_QUALITATIVE:
            break;
        case Constraint::TEMPORAL_QUANTITATIVE:
            break;
        case Constraint::MODEL:
           return apply( dynamic_pointer_cast<constraints::ModelConstraint>(constraint), transportNetwork);
        default:
           break;
    }
}

void MissionConstraintManager::apply(const shared_ptr<constraints::ModelConstraint>& constraint, TransportNetwork& transportNetwork)
{
    // Variable derived from solver
    Gecode::IntVarArray& roleUsage = transportNetwork.mRoleUsage;
    const Role::List& roles = transportNetwork.mRoles;
    FluentTimeResource::List& allRequirements = transportNetwork.mResourceRequirements;

    const owlapi::model::IRI& roleModel = constraint->getModel();

    FluentTimeResource::List affectedRequirements = findAffected(constraint, allRequirements);

    using namespace templ::constraints;
    switch(constraint->getModelConstraintType())
    {
        case ModelConstraint::MIN:
            MissionConstraints::min(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel,
                    constraint->getValue()
                    );
            break;
        case ModelConstraint::MAX:
            MissionConstraints::max(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel,
                    constraint->getValue()
                    );
            break;
        case ModelConstraint::ALL_DISTINCT:
            MissionConstraints::allDistinct(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel);
            break;
        case ModelConstraint::MIN_DISTINCT:
            MissionConstraints::minDistinct(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel,
                    constraint->getValue()
                    );
            break;
        case ModelConstraint::MAX_DISTINCT:
            MissionConstraints::maxDistinct(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel,
                    constraint->getValue()
                    );
            break;
        case ModelConstraint::ALL_EQUAL:
            MissionConstraints::allEqual(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel);
            break;
        case ModelConstraint::MIN_EQUAL:
            MissionConstraints::minEqual(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel,
                    constraint->getValue());
            break;
        case ModelConstraint::MAX_EQUAL:
            MissionConstraints::maxEqual(transportNetwork,
                    roleUsage, roles, allRequirements, affectedRequirements,
                    roleModel,
                    constraint->getValue());
            break;
        case ModelConstraint::MIN_FUNCTION:
            MissionConstraints::addResourceRequirement(
                    transportNetwork.mResources,
                    allRequirements,
                    affectedRequirements,
                    organization_model::Resource(constraint->getModel()),
                    transportNetwork.mpMission->getOrganizationModelAsk());

            break;
        case ModelConstraint::MIN_PROPERTY:
        {
            using namespace organization_model;
            Resource resource( constraint->getModel() );
            // Add the constraint to increase the transport capacity to
            // cover for the existing delta
            PropertyConstraint::Set constraints;
            PropertyConstraint propertyConstraint( constraint->getProperty(),
                    PropertyConstraint::GREATER_EQUAL, constraint->getValue());
            constraints.insert(propertyConstraint);
            resource.setPropertyConstraints(constraints);

            MissionConstraints::addResourceRequirement(
                    transportNetwork.mResources,
                    allRequirements,
                    affectedRequirements,
                    resource,
                    transportNetwork.mpMission->getOrganizationModelAsk());
            break;
        }
        case ModelConstraint::MAX_PROPERTY:
        {
            using namespace organization_model;
            Resource resource( constraint->getModel() );
            // Add the constraint to increase the transport capacity to
            // cover for the existing delta
            PropertyConstraint::Set constraints;
            PropertyConstraint propertyConstraint( constraint->getProperty(),
                    PropertyConstraint::LESS_EQUAL, constraint->getValue());
            constraints.insert(propertyConstraint);
            resource.setPropertyConstraints(constraints);

            MissionConstraints::addResourceRequirement(
                    transportNetwork.mResources,
                    allRequirements,
                    affectedRequirements,
                    resource,
                    transportNetwork.mpMission->getOrganizationModelAsk());
            break;
        }
        case ModelConstraint::MAX_FUNCTION:
        default:
            throw std::runtime_error("templ::solvers::csp::MissionConstraintManager::apply: " + ModelConstraint::TypeTxt[ constraint->getModelConstraintType() ]
                    + " is currently not supported");
    }
}

FluentTimeResource::List MissionConstraintManager::findAffected(const shared_ptr<constraints::ModelConstraint>& constraint, const FluentTimeResource::List& ftrs)
{
    FluentTimeResource::List affected;

    const std::vector<SpaceTime::SpaceIntervalTuple>& spaceIntervals = constraint->getSpaceIntervalTuples();
    for(const SpaceTime::SpaceIntervalTuple& spaceInterval : spaceIntervals)
    {
        for(const FluentTimeResource& ftr : ftrs)
        {
            if(ftr.getLocation() == spaceInterval.first())
            {
                if(ftr.getInterval() == spaceInterval.second())
                {
                    affected.push_back(ftr);
                }
            }
        }
    }
    if(affected.empty())
    {
        throw std::invalid_argument("templ::solvers::csp::MissionConstraintManager::findAffected:"
                " failed to identify affected requirements for constraint: " + constraint->toString());
    }

    return affected;
}


std::vector<SpaceTime::SpaceIntervalTuple> MissionConstraintManager::mapToSpaceTime(const FluentTimeResource::List& ftrs)
{
    std::vector<SpaceTime::SpaceIntervalTuple> tuples;
    for(const FluentTimeResource& ftr : ftrs)
    {
        SpaceTime::SpaceIntervalTuple tuple(ftr.getLocation(), ftr.getInterval());
        tuples.push_back(tuple);
    }
    return tuples;
}

SpaceTime::SpaceIntervalTuple MissionConstraintManager::mapToSpaceTime(const FluentTimeResource& ftr)
{
    return SpaceTime::SpaceIntervalTuple(ftr.getLocation(), ftr.getInterval());
}

} // end namespace csp
} // end namespace solvers
} // end namespace templ
