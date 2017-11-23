#include "MissionConstraint.hpp"
#include <sstream>

namespace templ {

std::map<MissionConstraint::Type, std::string> MissionConstraint::TypeTxt =  {
    { MissionConstraint::MIN_DISTINCT, "min-distinct"},
    { MissionConstraint::MAX_DISTINCT, "max-distinct"},
    { MissionConstraint::ALL_DISTINCT, "all-distinct"},
    { MissionConstraint::MIN_EQUAL,    "min-equal"},
    { MissionConstraint::MAX_EQUAL,    "max-equal"},
    { MissionConstraint::MIN_FUNCTION, "min-function"},
    { MissionConstraint::MAX_FUNCTION, "max-function"}
};

MissionConstraint::MissionConstraint()
    : mType(UNKNOWN)
    , mValue(0)
{}

MissionConstraint::MissionConstraint(Type type,
            const owlapi::model::IRI& model,
            const std::vector<SpaceTime::SpaceIntervalTuple>& affectedSpaceIntervals,
            uint32_t value,
            const owlapi::model::IRI& property)
    : mType(type)
    , mModel(model)
    , mSpaceIntervalTuples(affectedSpaceIntervals)
    , mValue(value)
    , mProperty(property)
{
}

std::string MissionConstraint::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << TypeTxt[mType] << ":" << std::endl;
    ss << hspace << "    model: " << mModel.toString() << std::endl;
    if(mType != ALL_DISTINCT)
    {
        ss << hspace << "    value: " << mValue << std::endl;
        if(mType == MIN_FUNCTION || mType == MAX_FUNCTION)
        {
            ss << hspace << "    property: " << mValue << std::endl;
        }
    }
    ss << hspace << "    space-time:" << std::endl;
    for(const SpaceTime::SpaceIntervalTuple& tuple : mSpaceIntervalTuples)
    {
        ss << tuple.toString(indent + 8);
    }
    return ss.str();
}

MissionConstraint::Type MissionConstraint::getTypeFromTxt(const std::string& txt)
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

    throw std::invalid_argument("templ::MissionConstraint::getTypeFromTxt: could not find type for '" + txt + "'");
}

} // end namespace templ
