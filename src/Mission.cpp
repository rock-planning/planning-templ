#include "Mission.hpp"
#include <boost/lexical_cast.hpp>

#include <owlapi/Vocabulary.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>
#include <templ/solvers/temporal/QualitativeTemporalConstraintNetwork.hpp>
#include <templ/symbols/object_variables/LocationCardinality.hpp>
#include <templ/symbols/object_variables/LocationNumericAttribute.hpp>

namespace templ {

namespace pa = solvers::temporal::point_algebra;

Mission::Mission(organization_model::OrganizationModel::Ptr om, const std::string& name)
    : mpTemporalConstraintNetwork(new solvers::temporal::QualitativeTemporalConstraintNetwork())
    , mpRelations(graph_analysis::BaseGraph::getInstance())
    , mpOrganizationModel(om)
    , mAsk(om)
    , mName(name)
    , mpLogger(new Logger())
{}

Mission::Mission(const Mission& other)
    : mpTemporalConstraintNetwork()
    , mpRelations()
    , mpOrganizationModel(other.mpOrganizationModel)
    , mAsk(other.mAsk)
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
    , mScenarioFile(other.mScenarioFile)
    , mpLogger(other.mpLogger)
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

void Mission::setOrganizationModel(organization_model::OrganizationModel::Ptr organizationModel)
{
    mpOrganizationModel = organizationModel;
    mAsk = organization_model::OrganizationModelAsk(organizationModel);
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
    mAsk = organization_model::OrganizationModelAsk(mpOrganizationModel, mModelPool, true /*functional saturation bound*/);
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

solvers::temporal::point_algebra::TimePoint::Ptr Mission::getTimePoint(const std::string& name) const
{
    using namespace solvers::temporal::point_algebra;

    graph_analysis::VertexIterator::Ptr it = mpTemporalConstraintNetwork->getVariableIterator();
    while(it->next())
    {
        QualitativeTimePoint::Ptr t = dynamic_pointer_cast<QualitativeTimePoint>(it->current());
        if(t && t->getLabel() == name)
        {
            return t;
        }
    }
    throw std::invalid_argument("templ::Mission::getTimePoint: timepoint with label '" + name + "'"
            " not found");
}

solvers::temporal::point_algebra::TimePoint::Ptr Mission::getOrCreateTimePoint(const std::string& name) const
{
    using namespace solvers::temporal::point_algebra;
    TimePoint::Ptr timepoint;
    try {
        return getTimePoint(name);
    } catch(const std::invalid_argument& e)
    {
        try {
            if(name == "inf")
            {
                return TimePoint::create(std::numeric_limits<uint64_t>::infinity(), std::numeric_limits<uint64_t>::infinity());
            } else {
                uint64_t exactTime = boost::lexical_cast<uint64_t>(name);
                return TimePoint::create(exactTime, exactTime);
            }
        } catch(const std::bad_cast& e)
        {
            return TimePoint::create(name);
        }
    }

    throw std::runtime_error("templ::Mission::getOrCreateTimePoint: timepoint with label/value '" + name + "' could neither be found nor created");
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
    addConstant(location);
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

    addConstant(location);
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
        pa::QualitativeTimePointConstraint::Type constraint)
{
    LOG_DEBUG_S << "Adding temporal constraint: " << t1->toString() << " --> " << t2->toString() << pa::QualitativeTimePointConstraint::TypeTxt[constraint];
    return mpTemporalConstraintNetwork->addQualitativeConstraint(t1, t2, constraint);
}

void Mission::addConstraint(const solvers::Constraint::Ptr& constraint)
{
    using namespace solvers::temporal::point_algebra;
    QualitativeTimePointConstraint::Ptr timeConstraint = dynamic_pointer_cast<QualitativeTimePointConstraint>(constraint);
    if(timeConstraint)
    {
        mpTemporalConstraintNetwork->addConstraint(dynamic_pointer_cast<solvers::Constraint>(timeConstraint));
    } else {
        mConstraints.push_back(constraint);
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
        ss << "        " << (*pit)->toString() << std::endl;
    }

    return ss.str();
}

void Mission::validateAvailableResources() const
{
    for(auto pair : mModelPool)
    {
        if(pair.second > 0)
            return;
    }

    throw std::runtime_error("templ::Mission: mission has no available resources specified (not present or max cardinalities sum up to 0) -- \ndefined : model pool is "
            + mModelPool.toString());
}


// Get special sets of constants
std::vector<symbols::constants::Location::Ptr> Mission::getLocations() const
{
    using namespace symbols;
    std::vector<constants::Location::Ptr> locations;
    std::set<Constant::Ptr>::const_iterator cit = mConstants.begin();
    for(; cit != mConstants.end(); ++cit)
    {
        const Constant::Ptr& constant = *cit;
        if(constant->getConstantType() == Constant::LOCATION)
        {
            locations.push_back( dynamic_pointer_cast<constants::Location>(constant) );
        }
    }
    return locations;
}


std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> Mission::getOrderedTimepoints() const
{
    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints = getTimepoints();
    mpTemporalConstraintNetwork->sort(timepoints);
    return timepoints;
}

std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> Mission::getTimepoints() const
{
    using namespace graph_analysis;
    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints;

    VertexIterator::Ptr vertexIt = mpTemporalConstraintNetwork->getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        solvers::temporal::point_algebra::TimePoint::Ptr tp =
            dynamic_pointer_cast<solvers::temporal::point_algebra::TimePoint>(vertexIt->current());
        timepoints.push_back(tp);
    }
    return timepoints;
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

graph_analysis::Edge::Ptr Mission::addRelation(graph_analysis::Vertex::Ptr source,
        const std::string& label,
        graph_analysis::Vertex::Ptr target)
{
    using namespace graph_analysis;
    Edge::Ptr edge(new Edge(source, target, label));
    mpRelations->addEdge(edge);
    return edge;
}

} // end namespace templ
