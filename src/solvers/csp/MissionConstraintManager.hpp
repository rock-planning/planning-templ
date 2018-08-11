#ifndef TEMPL_SOLVERS_CSP_MISSION_CONSTRAINT_MANAGER_HPP
#define TEMPL_SOLVERS_CSP_MISSION_CONSTRAINT_MANAGER_HPP

#include "../../SharedPtr.hpp"
#include "../../Constraint.hpp"
#include "../FluentTimeResource.hpp"
#include "../../SpaceTime.hpp"

namespace templ {
namespace constraints {
    class ModelConstraint;
} // end namespace constraints
} // end namespace templ

namespace templ {
namespace solvers {
namespace csp {

class TransportNetwork;

/**
 * \class MissionConstraintManager
 * \brief Manage the application of constraints to solvers
 * \details The mission constraint manager applies the constraints such as
 * templ::constraints::ModelConstraint to a given CSP
 */
class MissionConstraintManager
{
public:
    /**
     * Apply a list of generic constraints to a TransportNetowkr
     * \param constraint list Constraints to apply
     * \param transportNetwork TransportNetwork to apply the constraint to
     */
    static void apply(const Constraint::PtrList& constraints,
            TransportNetwork& transportNetwork);
    /**
     * Apply a generic constraint to a TransportNetwork
     * \param constraint Constraint to apply
     * \param transportNetwork TransportNetwork to apply the constraint to
     */
    static void apply(const Constraint::Ptr& constraint, TransportNetwork& transportNetwork);

    /**
     * Apply a constraints::ModelConstraint to a TransportNetwork
     * \param constraint Constraint to apply
     * \param transportNetwork TransportNetwork to apply the constraint to
     */
    static void apply(const shared_ptr<constraints::ModelConstraint>& constraint, TransportNetwork& transportNetwork);

    /**
     * Utility function to convert FluentTimeResource::List to SpaceTime (as
     * used in a Constraint
     * \param ftrs List of FluentTimeResource to be converted
     * \todo Find better place for this functionality
     */
    static std::vector<SpaceTime::SpaceIntervalTuple> mapToSpaceTime(const FluentTimeResource::List& ftrs);

    /**
     * Utility function to convert FluentTimeResource to SpaceTime (as
     * used in a Constraint
     * \param ftr FluentTimeResource to be converted
     * \todo Find better place for this functionality
     */
    static SpaceTime::SpaceIntervalTuple mapToSpaceTime(const FluentTimeResource& ftr);

private:
    /** Find all FluentTimeResource which are related to a Constraint
     * \param constraint Constraint that contains the requirement references
     * (which can point to either a location or spatio-time intervals)
     * \param ftrs List of fluent time resources that have to be considered for
     * this mission
     * \todo Find better place for this functionality
     * \return List of FluentTimeResource which contains only the one related to the constraint
     */
    static FluentTimeResource::Set findAffected(const shared_ptr<constraints::ModelConstraint>& constraint, const FluentTimeResource::List& ftrs);

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_CSP_MISSION_CONSTRAINT_MANAGER_HPP
