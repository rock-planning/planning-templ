#include "Mission.hpp"
#include <boost/lexical_cast.hpp>

#include <owlapi/Vocabulary.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>
#include <templ/symbols/object_variables/LocationCardinality.hpp>

namespace templ {

namespace pa = solvers::temporal::point_algebra;

Mission::Mission(const std::string& name)
    : mpTemporalConstraintNetwork(new solvers::temporal::QualitativeTemporalConstraintNetwork())
    , mpOrganizationModel(new organization_model::OrganizationModel())
    , mAsk(mpOrganizationModel)
    , mName(name)
{}

Mission::Mission(organization_model::OrganizationModel::Ptr om, const std::string& name)
    : mpTemporalConstraintNetwork(new solvers::temporal::QualitativeTemporalConstraintNetwork())
    , mpOrganizationModel(om)
    , mAsk(om)
    , mName(name)
{}

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

void Mission::prepare()
{
    using namespace solvers::temporal;

    validateAvailableResources();

    std::vector<PersistenceCondition::Ptr>::const_iterator cit =  mPersistenceConditions.begin();
    for(;cit != mPersistenceConditions.end(); ++cit)
    {
        PersistenceCondition::Ptr pc = *cit;
        Interval interval(pc->getFromTimePoint(), pc->getToTimePoint(), point_algebra::TimePointComparator(mpTemporalConstraintNetwork));

        mTimeIntervals.insert(interval);
    }
}

solvers::temporal::point_algebra::TimePoint::Ptr Mission::getTimePoint(const std::string& name) const
{
    using namespace solvers::temporal::point_algebra;

    graph_analysis::VertexIterator::Ptr it = mpTemporalConstraintNetwork->getVariableIterator();
    while(it->next())
    {
        QualitativeTimePoint::Ptr t = boost::dynamic_pointer_cast<QualitativeTimePoint>(it->current());
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

void Mission::addResourceLocationCardinalityConstraint(
            const symbols::constants::Location::Ptr& location,
            const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
            const solvers::temporal::point_algebra::TimePoint::Ptr& toTp,
            const owlapi::model::IRI& resourceModel,
            uint32_t cardinality,
            owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type
)
{
    using namespace owlapi;

    if(!mpOrganizationModel)
    {
        throw std::runtime_error("templ::Mission::addConstraint: mission has not been initialized with organization model");
    }

    //// Retrieve the type of the resource model
    //owlapi::model::IRI type;
    //if(mAsk.ontology().isSubClassOf(resourceModel, vocabulary::OM::Service()) )
    //{
    //    mInvolvedServices.insert(resourceModel);
    //    type = vocabulary::OM::Service();
    //} else if(mAsk.ontology().isSubClassOf(resourceModel, vocabulary::OM::Actor()) )
    //{
    //    mInvolvedActors.insert(resourceModel);
    //    type = vocabulary::OM::Actor();
    //} else {
    //    throw std::invalid_argument("templ::Mission::addConstraint: unsupported resource type for '" +
    //            resourceModel.toString() + "' -- supported are: " + 
    //            vocabulary::OM::Service().toString() + ", and " +
    //            vocabulary::OM::Actor().toString());
    //}

    mRequestedResources.insert(resourceModel);

    using namespace ::templ::symbols;
    // Make sure constant is known
    addConstant(location);
    ObjectVariable::Ptr locationCardinality(new object_variables::LocationCardinality(location, cardinality, type));
    mObjectVariables.insert(locationCardinality);

    // the combination of resource, location and cardinality represents a state variable
    // which needs to be translated into resource based state variables
    symbols::StateVariable rloc(ObjectVariable::TypeTxt[ObjectVariable::LOCATION_CARDINALITY],
            resourceModel.toString());

    addConstraint(rloc, locationCardinality, fromTp, toTp);
}

void Mission::addConstraint(const symbols::StateVariable& stateVariable,
        const symbols::ObjectVariable::Ptr& objectVariable,
        const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
        const solvers::temporal::point_algebra::TimePoint::Ptr& toTp)
{
    using namespace solvers::temporal;
    PersistenceCondition::Ptr persistenceCondition = PersistenceCondition::getInstance(stateVariable,
            objectVariable,
            fromTp,
            toTp);

    mpTemporalConstraintNetwork->addQualitativeConstraint(fromTp, toTp, pa::QualitativeTimePointConstraint::LessOrEqual);
    mPersistenceConditions.push_back(persistenceCondition);
}

void Mission::addTemporalConstraint(const pa::TimePoint::Ptr& t1,
        const pa::TimePoint::Ptr& t2,
        pa::QualitativeTimePointConstraint::Type constraint)
{
    mpTemporalConstraintNetwork->addQualitativeConstraint(t1, t2, constraint);
}

void Mission::addConstraint(const solvers::Constraint::Ptr& constraint)
{
    using namespace solvers::temporal::point_algebra;
    QualitativeTimePointConstraint::Ptr timeConstraint = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(constraint);
    if(timeConstraint)
    {
        mpTemporalConstraintNetwork->addConstraint(boost::dynamic_pointer_cast<solvers::Constraint>(timeConstraint));
    } else {
        mConstraints.push_back(constraint);
    }
}

std::string Mission::toString() const
{
    std::stringstream ss;
    ss << "Mission: " << mName << std::endl;
    ss << mModelPool.toString();
    return ss.str();
}

void Mission::validateAvailableResources() const
{
    for(auto pair : mModelPool)
    {
        if(pair.second > 0)
            return;
    }

    throw std::runtime_error("templ::Mission: mission has no available resources specified (not present or max cardinalities sum up to 0) -- \ndefined "
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
            locations.push_back( boost::dynamic_pointer_cast<constants::Location>(constant) );
        }
    }
    return locations;
}


std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> Mission::getTimepoints() const
{
    using namespace graph_analysis;
    std::vector<solvers::temporal::point_algebra::TimePoint::Ptr> timepoints;

    VertexIterator::Ptr vertexIt = mpTemporalConstraintNetwork->getGraph()->getVertexIterator();
    while(vertexIt->next())
    {
        solvers::temporal::point_algebra::TimePoint::Ptr tp =
            boost::dynamic_pointer_cast<solvers::temporal::point_algebra::TimePoint>(vertexIt->current());
        timepoints.push_back(tp);
    }
    return timepoints;
}

void Mission::addConstant(const symbols::Constant::Ptr& constant)
{
    mConstants.insert(constant);
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

} // end namespace templ
