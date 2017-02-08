#include "LocationNumericAttribute.hpp"

namespace templ {
namespace symbols {
namespace object_variables {

LocationNumericAttribute::LocationNumericAttribute(const constants::Location::Ptr& location,
        const owlapi::model::IRI& numericAttribute,
        int32_t minInclusive,
        int32_t maxInclusive)
    : ObjectVariable("unknown", LOCATION_NUMERIC_ATTRIBUTE)
    , mNumericAttribute(numericAttribute)
    , mMinInclusive(minInclusive)
    , mMaxInclusive(maxInclusive)
{
}

LocationNumericAttribute::~LocationNumericAttribute()
{}

bool LocationNumericAttribute::equals(const Symbol::Ptr& symbol) const
{
    LocationNumericAttribute::Ptr other = dynamic_pointer_cast<LocationNumericAttribute>(symbol);
    if(other)
    {
        return mLocation == other->mLocation &&
            mNumericAttribute == other->mNumericAttribute &&
            mMinInclusive == other->mMinInclusive &&
            mMaxInclusive == other->mMaxInclusive;
    }
    return false;
}

std::string LocationNumericAttribute::toString() const
{
    std::stringstream ss;
    ss << ObjectVariable::toString() << std::endl;
    ss <<    " location:    " << mLocation->toString() << std::endl;
    ss <<    " numericAttribute: " << mNumericAttribute.toString() << std::endl;
    ss <<    "    minInclusive: " << mMinInclusive << std::endl;
    ss <<    "    maxInclusive: " << mMaxInclusive << std::endl;
    return ss.str();

}

} // end namespace object_variables
} // end namespace symbols
} // end templ
