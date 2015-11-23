#ifndef TEMPL_SOLVERS_CSP_RESOLVER_HPP
#define TEMPL_SOLVERS_CSP_RESOLVER_HPP

#include <boost/shared_ptr.hpp>
#include <templ/solvers/csp/FluentTimeResource.hpp>
#include <templ/solvers/csp/RoleDistribution.hpp>
#include <templ/solvers/csp/ModelDistribution.hpp>

namespace templ {

class MissionPlanner;

namespace solvers {
namespace csp {

class Resolver
{
public:
    typedef boost::shared_ptr<Resolver> Ptr;

    enum Type { UNKNOWN, MODEL_DISTRIBUTION, ROLE_DISTRIBUTION };

    Resolver(const Type& type = UNKNOWN);
    virtual ~Resolver() {}

    Type getType() const { return mType; }

    /**
     * Allow to modifiy the internal state of the mission planner
     */
    virtual void apply(MissionPlanner* missionPlanner) { throw std::runtime_error("templ::solvers::csp::Resolver not implemented"); }

private:
    Type mType;
};

class RoleAddDistinction : public Resolver
{
public:
    RoleAddDistinction(const FluentTimeResource& fts0, 
            const FluentTimeResource& fts1,
            const owlapi::model::IRI& model,
            uint32_t addDelta,
            const RoleDistribution::Solution& solution);
    
    virtual ~RoleAddDistinction(){}

    void apply(MissionPlanner* missionPlanner);

private:
    FluentTimeResource mFts0;
    FluentTimeResource mFts1;
    owlapi::model::IRI mModel;
    uint32_t mAdd;
    RoleDistribution::Solution mSolution;
};

class FunctionalityRequest : public Resolver
{
public:
    FunctionalityRequest(const FluentTimeResource& fts,
            const owlapi::model::IRI& model);

    virtual ~FunctionalityRequest() {}

    void apply(MissionPlanner* missionPlanner);

private:
    FluentTimeResource mFts;
    owlapi::model::IRI mModel;

};

} // end namespace csp
} // end namespace solvers
} // end namespace templ

#endif // TEMPL_SOLVERS_CSP_RESOLVER_HPP
