#include "Mission.hpp"
#include <boost/lexical_cast.hpp>
#include <iterator>

#include <owlapi/Vocabulary.hpp>
#include <organization_model/Algebra.hpp>
#include "solvers/csp/FluentTimeResource.hpp"
#include "solvers/temporal/point_algebra/TimePointComparator.hpp"
#include "solvers/temporal/QualitativeTemporalConstraintNetwork.hpp"
#include "symbols/object_variables/LocationCardinality.hpp"
#include "symbols/object_variables/LocationNumericAttribute.hpp"

namespace templ {

namespace pa = solvers::temporal::point_algebra;

Mission::Mission(organization_model::OrganizationModel::Ptr om, const std::string& name)
    : mpTemporalConstraintNetwork(new solvers::temporal::QualitativeTemporalConstraintNetwork())
    , mpRelations(graph_analysis::BaseGraph::getInstance())
    , mpOrganizationModel(om)
    , mOrganizationModelAsk(om)
    , mName(name)
    , mpTransferLocation(new symbols::constants::Location("transfer-location"))
    , mpLogger(new Logger())
{
    requireConstant(mpTransferLocation);

    // add all functionalities (which could be requested)
    //
    owlapi::model::IRIList list = mOrganizationModelAsk.ontology().allSubClassesOf(organization_model::vocabulary::OM::Functionality());
    mRequestedResources.insert(mRequestedResources.begin(), list.begin(), list.end());
}

Mission::Mission(const Mission& other)
    : mpTemporalConstraintNetwork()
    , mpRelations()
    , mpOrganizationModel(other.mpOrganizationModel)
    , mOrganizationModelAsk(other.mOrganizationModelAsk)
    , mName(other.mName)
    , mModelPool(other.mModelPool)
    , mRoles(other.mRoles)
    , mModels(other.mModels)
    , mPersistenceConditions(other.mPersistenceConditions)
    , mConstraints(other.mConstraints)
    , mRequestedResources(other.mRequestedResources)
    , mTimeIntervals(other.mTimeIntervals)
    , mObjectVariables(other.mObjectVariables)
    , mConstants(other.mConstants)
    , mConstantsUse(other.mConstantsUse)
    , mpTransferLocation(other.mpTransferLocation)
    , mScenarioFile(other.mScenarioFile)
    , mpLogger(other.mpLogger)
    , mDataPropertyAssignments(other.mDataPropertyAssignments)
{

    if(other.mpRelations)
    {
        mpRelations = other.mpRelations->clone();
    }

    if(other.mpTemporalConstraintNetwork)
    {
        solvers::ConstraintNetwork::Ptr constraintNetwork = other.mpTemporalConstraintNetwork->clone();
        mpTemporalConstraintNetwork = dynamic_pointer_cast<solvers::temporal::QualitativeTemporalConstraintNetwork>(constraintNetwork);
    }
}

void Mission::setOrganizationModel(const organization_model::OrganizationModel::Ptr& organizationModel)
{
    mpOrganizationModel = organizationModel;
    mOrganizationModelAsk = organization_model::OrganizationModelAsk(organizationModel);
}

void Mission::applyOrganizationModelOverrides()
{
    DataPropertyAssignment::apply(mpOrganizationModel, mDataPropertyAssignments);
}

void Mission::setAvailableResources(const organization_model::ModelPool& modelPool)
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

    // Update the ask object based on the model pool and applying the functional
    // saturation bound
    assert(mpOrganizationModel);
    mOrganizationModelAsk = organization_model::OrganizationModelAsk(mpOrganizationModel, mModelPool, true /*functional saturation bound*/);
}

symbols::ObjectVariable::Ptr Mission::getObjectVariable(const std::string& name, symbols::ObjectVariable::Type type) const
{
    using namespace ::templ::symbols;
    std::set<ObjectVariable::Ptr>::const_iterator cit = mObjectVariables.begin();
    for(; cit != mObjectVariables.end(); ++cit)
    {
        ObjectVariable::Ptr variable = *cit;
        if(type != ObjectVariable::UNKNOWN && variable->getObjectVariableType() == type && variable->getInstanceName() == name)
        {
            return variable;
        }
    }
    throw std::invalid_argument("templ::Mission::getObjectVariable: variable with name '" + name + "'"
            " and type '" + ObjectVariable::TypeTxt[type] + "' not found");
}

symbols::ObjectVariable::Ptr Mission::getOrCreateObjectVariable(const std::string& name, symbols::ObjectVariable::Type type) const
{
    using namespace ::templ::symbols;

    ObjectVariable::Ptr variable;
    try {
         variable = getObjectVariable(name, type);
    } catch(const std::invalid_argument& e)
    {
        variable = ObjectVariable::getInstance(name, type);
    }
    return variable;
}

void Mission::prepareTimeIntervals()
{
    using namespace solvers::temporal;

    if(!mpTemporalConstraintNetwork->isConsistent())
    {
        throw std::runtime_error("templ::Mission: provided temporal constraint network is not consistent");
    }

    mTimeIntervals.clear();
    std::vector<PersistenceCondition::Ptr>::const_iterator cit =  mPersistenceConditions.begin();
    for(;cit != mPersistenceConditions.end(); ++cit)
    {
        PersistenceCondition::Ptr pc = *cit;
        Interval interval(pc->getFromTimePoint(), pc->getToTimePoint(), point_algebra::TimePointComparator(mpTemporalConstraintNetwork));

        std::vector<Interval>::const_iterator tit = std::find(mTimeIntervals.begin(), mTimeIntervals.end(), interval);
        if(tit == mTimeIntervals.end())
        {
            mTimeIntervals.push_back(interval);
        } else {
            LOG_DEBUG_S << "TimeInterval: " << tit->toString() << " already in list";
        }
    }
}

void Mission::validateForPlanning() const
{
    if(!mpOrganizationModel)
    {
        throw std::runtime_error("templ::Mission::validate no organization model hase been set.");
    }

    validateAvailableResources();
    if(!mpTemporalConstraintNetwork->isConsistent())
    {
        throw std::runtime_error("templ::Mission::validate provided temporal constraint network is not consistent");
    }

    if(mTimeIntervals.empty())
    {
        throw std::runtime_error("templ::Mission::validate: no time intervals defined");
    }
}

solvers::temporal::TemporalAssertion::Ptr Mission::addResourceLocationCardinalityConstraint(
            const symbols::constants::Location::Ptr& location,
            const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
            const solvers::temporal::point_algebra::TimePoint::Ptr& toTp,
            const owlapi::model::IRI& resourceModel,
            uint32_t cardinality,
            owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type
)
{
    if(!mpOrganizationModel)
    {
        throw std::runtime_error("templ::Mission::addResourceLocationCardinalityConstraint: mission has not been initialized with organization model");
    }

    using namespace owlapi;
    using namespace ::templ::symbols;
    // Make sure constant is known
    requireConstant(location);
    ObjectVariable::Ptr locationCardinality(new object_variables::LocationCardinality(location, cardinality, type));

    // the combination of resource, location and cardinality represents a state variable
    // which needs to be translated into resource based state variables
    symbols::StateVariable rloc(ObjectVariable::TypeTxt[ObjectVariable::LOCATION_CARDINALITY],
            resourceModel.toString());

    solvers::temporal::TemporalAssertion::Ptr temporalAssertion = addTemporalAssertion(rloc, locationCardinality, fromTp, toTp);

    owlapi::model::IRIList::const_iterator rit =  std::find(mRequestedResources.begin(),
            mRequestedResources.end(),
            resourceModel);
    if(rit == mRequestedResources.end())
    {
        mRequestedResources.push_back(resourceModel);
    }

    mObjectVariables.insert(locationCardinality);
    return temporalAssertion;
}

solvers::temporal::TemporalAssertion::Ptr Mission::addResourceLocationNumericAttributeConstraint(
        const symbols::constants::Location::Ptr& location,
        const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
        const solvers::temporal::point_algebra::TimePoint::Ptr& toTp,
        const owlapi::model::IRI& resourceModel,
        const owlapi::model::IRI& attribute,
        int32_t minInclusive,
        int32_t maxInclusive
        )
{
    using namespace owlapi;
    using namespace ::templ::symbols;

    requireConstant(location);
    ObjectVariable::Ptr numericAttribute(new object_variables::LocationNumericAttribute(
                location,
                attribute,
                minInclusive,
                maxInclusive));

    symbols::StateVariable rloc(ObjectVariable::TypeTxt[ObjectVariable::LOCATION_NUMERIC_ATTRIBUTE],
            resourceModel.toString());

    solvers::temporal::TemporalAssertion::Ptr temporalAssertion = addTemporalAssertion(rloc,
            numericAttribute,
            fromTp,
            toTp);

    mObjectVariables.insert(numericAttribute);
    return temporalAssertion;
}

solvers::temporal::TemporalAssertion::Ptr Mission::addTemporalAssertion(const symbols::StateVariable& stateVariable,
        const symbols::ObjectVariable::Ptr& objectVariable,
        const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
        const solvers::temporal::point_algebra::TimePoint::Ptr& toTp)
{
    using namespace solvers::temporal;
    PersistenceCondition::Ptr persistenceCondition = PersistenceCondition::getInstance(stateVariable,
            objectVariable,
            fromTp,
            toTp);

    std::vector<solvers::temporal::PersistenceCondition::Ptr>::const_iterator pit = std::find_if(mPersistenceConditions.begin(), mPersistenceConditions.end(), [persistenceCondition](const PersistenceCondition::Ptr& p)
            {
                return *persistenceCondition == *p;
            });

    if(pit == mPersistenceConditions.end())
    {
        LOG_DEBUG_S << "Adding spatio-temporal constraint: " << stateVariable.toString()
            << " " << objectVariable->toString() << "@[" << fromTp->toString() << "--" << toTp->toString() <<"]" << std::endl;
        mPersistenceConditions.push_back(persistenceCondition);
    } else {
        LOG_DEBUG_S << "Already existing: " << (*pit)->toString();
        throw std::invalid_argument("templ::Mission::addTemporalAssertion: trying to re-add temporal assertion or"
                "trying to add redundant temporal assertion: '" + persistenceCondition->toString() + "'");
    }

    LOG_DEBUG_S << "Adding implicitly defined temporal constraint for time interval";
    mpTemporalConstraintNetwork->addQualitativeConstraint(fromTp, toTp, pa::QualitativeTimePointConstraint::Less);
    return persistenceCondition;
}

solvers::Constraint::Ptr Mission::addTemporalConstraint(const pa::TimePoint::Ptr& t1,
        const pa::TimePoint::Ptr& t2,
        pa::QualitativeTimePointConstraint::Type constraintType)
{
    LOG_DEBUG_S << "Adding temporal constraint: " << t1->toString() << " --> " << t2->toString() << pa::QualitativeTimePointConstraint::TypeTxt[constraintType];
    solvers::Constraint::Ptr constraint = mpTemporalConstraintNetwork->addQualitativeConstraint(t1, t2, constraintType);

    mConstraints.push_back(constraint);
    return constraint;
}

void Mission::addConstraint(const solvers::Constraint::Ptr& constraint)
{
    using namespace solvers::temporal::point_algebra;
    QualitativeTimePointConstraint::Ptr timeConstraint = dynamic_pointer_cast<QualitativeTimePointConstraint>(constraint);
    mConstraints.push_back(constraint);

    if(timeConstraint)
    {
        mpTemporalConstraintNetwork->addConstraint(dynamic_pointer_cast<solvers::Constraint>(timeConstraint));
    }
}

std::string Mission::toString() const
{
    std::stringstream ss;
    ss << "Mission: " << mName << std::endl;
    ss << mModelPool.toString();

    ss << "    PersistenceConditions " << std::endl;
    std::vector<solvers::temporal::PersistenceCondition::Ptr>::const_iterator pit = mPersistenceConditions.begin();
    for(; pit != mPersistenceConditions.end(); ++pit)
    {
        ss << (*pit)->toString(8) << std::endl;
    }

    return ss.str();
}

void Mission::validateAvailableResources() const
{
    for(const std::pair<owlapi::model::IRI, size_t>& pair : mModelPool)
    {
        if(pair.second > 0)
            return;
    }

    throw std::runtime_error("templ::Mission: mission has no available resources specified (not present or max cardinalities sum up to 0) -- \ndefined : model pool is "
            + mModelPool.toString());
}

std::vector<solvers::csp::FluentTimeResource> Mission::getResourceRequirements(const Mission::Ptr& mission)
{
    using namespace solvers::csp;

    if(mission->mTimeIntervals.empty())
    {
        throw std::runtime_error("Mission::getResourceRequirements: no time intervals available"
                " -- make sure you called prepareTimeIntervals() on the mission instance");
    }

    std::vector<FluentTimeResource> requirements;

    // Iterate over all existing persistence conditions
    // -- pick the ones relating to location-cardinality function
    using namespace templ::solvers::temporal;
    for(PersistenceCondition::Ptr p : mission->mPersistenceConditions)
    {
        symbols::StateVariable stateVariable = p->getStateVariable();
        if(stateVariable.getFunction() == symbols::ObjectVariable::TypeTxt[symbols::ObjectVariable::LOCATION_CARDINALITY] )
        {
            try {
                FluentTimeResource ftr = fromLocationCardinality( p, mission );
                requirements.push_back(ftr);
                LOG_DEBUG_S << ftr.toString();
            } catch(const std::invalid_argument& e)
            {
                LOG_WARN_S << e.what();
            }
        }
    }

    // If multiple requirement exists that have the same interval
    // they can be compacted into one requirement
    FluentTimeResource::compact(requirements, mission->mOrganizationModelAsk);

    // Sort the requirements based on the start timepoint, i.e. the from
    using namespace templ::solvers::temporal;
    point_algebra::TimePointComparator timepointComparator(mission->getTemporalConstraintNetwork());
    std::sort(requirements.begin(), requirements.end(), [&timepointComparator](const FluentTimeResource& a,const FluentTimeResource& b) -> bool
            {
                return timepointComparator.lessThan(a.getInterval().getFrom(), b.getInterval().getFrom());
            });
    return requirements;
}


// Get special sets of constants
std::vector<symbols::constants::Location::Ptr> Mission::getLocations(bool excludeUnused) const
{
    using namespace symbols;
    std::vector<constants::Location::Ptr> locations;
    std::set<Constant::Ptr>::const_iterator cit = mConstants.begin();
    for(; cit != mConstants.end(); ++cit)
    {
        const Constant::Ptr& constant = *cit;
        if(excludeUnused && mConstantsUse[constant] == 0)
        {
                continue;
        }

        if(constant->getConstantType() == Constant::LOCATION)
        {
            locations.push_back( dynamic_pointer_cast<constants::Location>(constant) );
        }
    }
    return locations;
}


std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> Mission::getTimepoints() const
{
    std::set<solvers::temporal::point_algebra::TimePoint::Ptr> uniqueTimepoints;
    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints;

    for(const solvers::temporal::Interval& interval : mTimeIntervals)
    {
        uniqueTimepoints.insert( interval.getFrom() );
        uniqueTimepoints.insert( interval.getTo() );
    }

    timepoints.insert(timepoints.begin(), uniqueTimepoints.begin(), uniqueTimepoints.end());
    solvers::temporal::QualitativeTemporalConstraintNetwork::Ptr qtcn = dynamic_pointer_cast<solvers::temporal::QualitativeTemporalConstraintNetwork>(mpTemporalConstraintNetwork);

    if(qtcn)
    {
        std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> tps = solvers::csp::TemporalConstraintNetwork::getSortedList(*qtcn);
        return tps;
    } else {
        mpTemporalConstraintNetwork->sort(timepoints);
    }
    return timepoints;
}

void Mission::requireConstant(const symbols::Constant::Ptr& constant)
{
    addConstant(constant);
    incrementConstantUse(constant);
}

uint32_t Mission::incrementConstantUse(const symbols::Constant::Ptr& constant)
{
    mConstantsUse[constant] += 1;
    return mConstantsUse[constant];
}

void Mission::addConstant(const symbols::Constant::Ptr& constant)
{
    using namespace symbols;
    std::set<Constant::Ptr>::const_iterator cit = std::find_if(mConstants.begin(), mConstants.end(), [constant](const symbols::Constant::Ptr& c)
            {
                return *constant == *c;
            });
    if(cit == mConstants.end())
    {
        mConstants.insert(constant);
    } else {
        if(*cit == constant)
        {
            // We found the constant -- nothing to do since it has already been
            // added
        } else {
            // We found the constant -- but pointers are different
            // This will lead to inconsitencies, so throw
            throw std::invalid_argument("templ::Mission::addConstant: constant '" +
                constant->toString() + "' already registered, but different pointer objects");
        }
    }
}

const symbols::Constant::Ptr& Mission::getConstant(const std::string& id, symbols::Constant::Type type)
{
    using namespace symbols;
    std::set<Constant::Ptr>::const_iterator cit = mConstants.begin();
    for(; cit != mConstants.end(); ++cit)
    {
        const Constant::Ptr& constant = *cit;
        if(type != symbols::Constant::UNKNOWN && constant->getConstantType() == type)
        {
            if(constant->getInstanceName() == id)
            {
                return constant;
            }
        }
    }
    throw std::invalid_argument("templ::Mission: no constant named '" + id + "' of type '" + symbols::Constant::TypeTxt[type] + "' known");
}

graph_analysis::Edge::Ptr Mission::addRelation(const graph_analysis::Vertex::Ptr& source,
        const std::string& label,
        const graph_analysis::Vertex::Ptr& target)
{
    using namespace graph_analysis;
    Edge::Ptr edge(new Edge(source, target, label));
    mpRelations->addEdge(edge);

    std::cout << "Adding relation from: " << source->toString() << " to " << target->toString() << std::endl;
    return edge;
}

solvers::csp::FluentTimeResource Mission::fromLocationCardinality(const solvers::temporal::PersistenceCondition::Ptr& p, const Mission::Ptr& mission)
{
    using namespace templ::solvers::temporal;
    point_algebra::TimePointComparator timepointComparator(mission->getTemporalConstraintNetwork());

    const symbols::StateVariable& stateVariable = p->getStateVariable();
    owlapi::model::IRI resourceModel(stateVariable.getResource());
    symbols::ObjectVariable::Ptr objectVariable = dynamic_pointer_cast<symbols::ObjectVariable>(p->getValue());
    symbols::object_variables::LocationCardinality::Ptr locationCardinality = dynamic_pointer_cast<symbols::object_variables::LocationCardinality>(objectVariable);

    Interval interval(p->getFromTimePoint(), p->getToTimePoint(), timepointComparator);
    std::vector<Interval>::const_iterator iit = std::find(mission->mTimeIntervals.begin(), mission->mTimeIntervals.end(), interval);
    if(iit == mission->mTimeIntervals.end())
    {
        throw std::runtime_error("templ::Mission::fromLocationCardinality: could not find interval: '" + interval.toString() + "'");
    }

    owlapi::model::IRIList::const_iterator sit = std::find(mission->mRequestedResources.begin(), mission->mRequestedResources.end(), resourceModel);
    if(sit == mission->mRequestedResources.end())
    {
        throw std::runtime_error("templ::Mission::fromLocationCardinality: could not find service: '" + resourceModel.toString() + "'");
    }

    symbols::constants::Location::Ptr location = locationCardinality->getLocation();

    std::vector<symbols::constants::Location::Ptr> locations = mission->getLocations();
    std::vector<symbols::constants::Location::Ptr>::const_iterator lit = std::find(locations.begin(), locations.end(), location);
    if(lit == locations.end())
    {
        throw std::runtime_error("templ::solvers::csp::TransportNetwork::getResourceRequirements: could not find location: '" + location->toString() + "'");
    }

    using namespace solvers::csp;

    // Map objects to numeric indices -- the indices can be mapped
    // backed using the mission they were created from
    uint32_t timeIndex = std::distance(mission->mTimeIntervals.cbegin(), iit);
    FluentTimeResource ftr(mission,
            (int) std::distance(mission->mRequestedResources.cbegin(), sit)
            , timeIndex
            , (int) std::distance(locations.cbegin(), lit)
    );

    owlapi::model::OWLOntologyAsk ask = mission->getOrganizationModelAsk().ontology();
    if(ask.isSubClassOf(resourceModel, organization_model::vocabulary::OM::Functionality()))
    {
        // retrieve upper bound
        ftr.maxCardinalities = mission->mOrganizationModelAsk.getFunctionalSaturationBound(resourceModel);

    } else if(ask.isSubClassOf(resourceModel, organization_model::vocabulary::OM::Actor()))
    {
        switch(locationCardinality->getCardinalityRestrictionType())
        {
            case owlapi::model::OWLCardinalityRestriction::MIN :
            {
                size_t min = ftr.minCardinalities.getValue(resourceModel, std::numeric_limits<size_t>::min());
                ftr.minCardinalities[ resourceModel ] = std::max(min, (size_t) locationCardinality->getCardinality());
                break;
            }
            case owlapi::model::OWLCardinalityRestriction::MAX :
            {
                size_t max = ftr.maxCardinalities.getValue(resourceModel, std::numeric_limits<size_t>::max());
                ftr.maxCardinalities[ resourceModel ] = std::min(max, (size_t) locationCardinality->getCardinality());
                break;
            }
            default:
                break;
        }
    } else {
        throw std::invalid_argument("templ::Mission::fromLocationCardinality: Unsupported state variable: " + resourceModel.toString());
    }

    ftr.maxCardinalities = organization_model::Algebra::max(ftr.maxCardinalities, ftr.minCardinalities);
    return ftr;
}

} // end namespace templ
