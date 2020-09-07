#include "ModelConstraint.hpp"
#include <sstream>
#include <moreorg/vocabularies/OM.hpp>

namespace templ {
namespace constraints {

std::map<ModelConstraint::Type, std::string> ModelConstraint::TypeTxt =  {
    { ModelConstraint::MIN,          "min" },
    { ModelConstraint::MAX,          "max" },
    { ModelConstraint::MIN_ACCESS,   "min-access" },
    { ModelConstraint::MAX_ACCESS,   "max-access" },
    { ModelConstraint::MIN_DISTINCT, "min-distinct"},
    { ModelConstraint::MAX_DISTINCT, "max-distinct"},
    { ModelConstraint::ALL_DISTINCT, "all-distinct"},
    { ModelConstraint::MIN_EQUAL,    "min-equal"},
    { ModelConstraint::MAX_EQUAL,    "max-equal"},
    { ModelConstraint::ALL_EQUAL,    "all-equal"},
    { ModelConstraint::MIN_FUNCTION, "min-function"},
    { ModelConstraint::MAX_FUNCTION, "max-function"},
    { ModelConstraint::MIN_PROPERTY, "min-property"},
    { ModelConstraint::MAX_PROPERTY, "max-property"}
};

ModelConstraint::ModelConstraint()
    : HyperConstraint(MODEL)
    , mType(UNKNOWN)
    , mValue(0)
{}

ModelConstraint::ModelConstraint(Type type,
            const owlapi::model::IRI& model,
            const std::vector<SpaceTime::SpaceIntervalTuple>& affectedSpaceIntervals,
            uint32_t value,
            const owlapi::model::IRI& property)
    : HyperConstraint(MODEL)
    , mType(type)
    , mModel(model)
    , mSpaceIntervalTuples(affectedSpaceIntervals)
    , mValue(value)
    , mProperty(property)
{
}

ModelConstraint::ModelConstraint(Type type,
            const owlapi::model::IRI& model,
            const SpaceTime::SpaceIntervalTuple& affectedSpaceInterval,
            uint32_t value,
            const owlapi::model::IRI& property)
    : HyperConstraint(MODEL)
    , mType(type)
    , mModel(model)
    , mValue(value)
    , mProperty(property)
{
    mSpaceIntervalTuples.push_back(affectedSpaceInterval);
}

bool ModelConstraint::operator==(const Constraint& _other) const
{
    const ModelConstraint* other = dynamic_cast<const ModelConstraint*>(&_other);
    if(other)
    {
        return Constraint::operator==(_other)
            && getSourceVertices() == other->getSourceVertices()
            && getTargetVertices() == other->getTargetVertices()
            && mType == other->mType
            && mModel == other->mModel
            && mSpaceIntervalTuples == other->mSpaceIntervalTuples
            && mValue == other->mValue
            && mProperty == other->mProperty;
    }
    return false;
}
std::string ModelConstraint::toString(uint32_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << TypeTxt[mType] << ":" << std::endl;
    ss << hspace << "    tags: " << getTagsAsString() << std::endl;
    ss << hspace << "    model: " << mModel.toString() << std::endl;
    if(! (mType == ALL_DISTINCT || mType == ALL_EQUAL) )
    {
        ss << hspace << "    value: " << mValue << std::endl;
        if(mType == MIN_PROPERTY || mType == MAX_PROPERTY)
        {
            ss << hspace << "    property: " << mValue << std::endl;
        }
    }
    if(mSpaceIntervalTuples.empty())
    {
        ss << hspace << "    space-time: n/a" << std::endl;
    } else {
        ss << hspace << "    space-time:" << std::endl;
        for(const SpaceTime::SpaceIntervalTuple& tuple : mSpaceIntervalTuples)
        {
            ss << tuple.toString(indent + 8);
        }
    }
    return ss.str();
}

ModelConstraint::Type ModelConstraint::getTypeFromTxt(const std::string& txt)
{
    std::map<Type, std::string>::const_iterator cit = std::find_if(TypeTxt.cbegin(), TypeTxt.cend(),
            [txt](const std::pair<Type, std::string>& other)
            {
                return other.second == txt;
            });
    if(cit != TypeTxt.end())
    {
        return cit->first;
    }

    throw std::invalid_argument("templ::constraints::ModelConstraint::getTypeFromTxt: could not find type for '" + txt + "'");
}

void ModelConstraint::validate(const owlapi::model::OWLOntologyAsk& ask) const
{
    if(!mModel.empty() && !ask.isSubClassOf(mModel, moreorg::vocabulary::OM::Resource()))
    {
        throw std::invalid_argument("templ::constraints::ModelConstraint::validate: "
            "resource of type '" + mModel.toString() + "' does not exist --");

    }
    if(!mProperty.empty() && !ask.isDataProperty(mProperty))
    {
        throw std::invalid_argument("templ::constraints::ModelConstraint::validate: "
            "data property of type '" + mProperty.toString() + "' does not exist");

    }
}

} // end namespace constraints
} // end namespace templ
