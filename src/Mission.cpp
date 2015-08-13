#include "Mission.hpp"
#include <boost/lexical_cast.hpp>

#include <owlapi/Vocabulary.hpp>
#include <templ/solvers/temporal/point_algebra/TimePointComparator.hpp>
#include <templ/object_variables/LocationCardinality.hpp>

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

ObjectVariable::Ptr Mission::getObjectVariable(const std::string& name, const std::string& type) const
{
    std::set<ObjectVariable::Ptr>::const_iterator cit = mObjectVariables.begin();
    for(; cit != mObjectVariables.end(); ++cit)
    {
        ObjectVariable::Ptr variable = *cit;
        if(variable->getTypeName() == type && variable->getInstanceName() == name)
        {
            return variable;
        }
    }
    throw std::invalid_argument("templ::Mission::getObjectVariable: variable with name '" + name + "'"
            " and type '" + type + "' not found");
}

ObjectVariable::Ptr Mission::getOrCreateObjectVariable(const std::string& name, const std::string& type) const
{
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
            const std::string& locationId,
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

    ObjectVariable::Ptr locationCardinality(new object_variables::LocationCardinality(locationId, cardinality, type));
    mObjectVariables.insert(locationCardinality);

    // the combination of resource, location and cardinality represents a state variable
    // which needs to be translated into resource based state variables
    StateVariable rloc(ObjectVariable::TypeTxt[ObjectVariable::LOCATION_CARDINALITY],
            resourceModel.toString());

    // Add to locations
    mLocations.insert(locationId);
    addConstraint(rloc, locationCardinality, fromTp, toTp);
}

void Mission::addConstraint(const StateVariable& stateVariable,
        const ObjectVariable::Ptr& objectVariable,
        const solvers::temporal::point_algebra::TimePoint::Ptr& fromTp,
        const solvers::temporal::point_algebra::TimePoint::Ptr& toTp)
{
    using namespace solvers::temporal;
    PersistenceCondition::Ptr persistenceCondition = PersistenceCondition::getInstance(stateVariable,
            objectVariable,
            fromTp,
            toTp);

    mpTemporalConstraintNetwork->addConstraint(fromTp, toTp, pa::QualitativeTimePointConstraint::LessOrEqual);
    mPersistenceConditions.push_back(persistenceCondition);
}

void Mission::addTemporalConstraint(const pa::TimePoint::Ptr& t1,
        const pa::TimePoint::Ptr& t2,
        pa::QualitativeTimePointConstraint::Type constraint)
{
    mpTemporalConstraintNetwork->addConstraint(t1, t2, constraint);
}

void Mission::addConstraint(const solvers::Constraint::Ptr& constraint)
{
    using namespace solvers::temporal::point_algebra;
    QualitativeTimePointConstraint::Ptr timeConstraint = boost::dynamic_pointer_cast<QualitativeTimePointConstraint>(constraint);
    if(timeConstraint)
    {
        mpTemporalConstraintNetwork->addConstraint(timeConstraint);
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

} // end namespace templ
