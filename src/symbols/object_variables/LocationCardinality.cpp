#include "LocationCardinality.hpp"
#include <sstream>

namespace templ {
namespace symbols {
namespace object_variables {

LocationCardinality::LocationCardinality(const constants::Location::Ptr& location,
        uint32_t cardinality,
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
    std::stringstream ss;
    ss << ObjectVariable::toString() << std::endl;
    ss <<    " location:    " << mLocation->toString() << std::endl;
    ss <<    " cardinality: " << mCardinality << std::endl;
    ss <<    " restriction: " << owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionTypeTxt[mCardinalityRestrictionType] << std::endl;
    return ss.str();
}

} // end namespace object_variables
} // end namespace symbols
} // end templ
