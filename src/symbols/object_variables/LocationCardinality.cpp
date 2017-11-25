#include "LocationCardinality.hpp"
#include <sstream>

namespace templ {
namespace symbols {
namespace object_variables {

LocationCardinality::LocationCardinality(const constants::Location::Ptr& location,
        size_t cardinality,
        owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type
)
    : ObjectVariable("unknown", LOCATION_CARDINALITY)
    , mLocation(location)
    , mCardinality(cardinality)
    , mCardinalityRestrictionType(type)
{}

LocationCardinality::~LocationCardinality() {}

bool LocationCardinality::equals(const Symbol::Ptr& symbol) const
{
    LocationCardinality::Ptr other = dynamic_pointer_cast<LocationCardinality>(symbol);
    if(other)
    {
        return mLocation == other->mLocation &&
            mCardinality == other->mCardinality &&
            mCardinalityRestrictionType == other->mCardinalityRestrictionType;
    }
    return false;
}

std::string LocationCardinality::toString() const
{
    return toString(0);
}

std::string LocationCardinality::toString(size_t indent) const
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << ObjectVariable::toString(indent) << std::endl;
    ss << hspace << "        location:    " << mLocation->toString() << std::endl;
    ss << hspace << "        cardinality: " << mCardinality << std::endl;
    ss << hspace << "        restriction: " << owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionTypeTxt[mCardinalityRestrictionType];
    return ss.str();
}

} // end namespace object_variables
} // end namespace symbols
} // end templ
