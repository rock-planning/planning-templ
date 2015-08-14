#ifndef TEMPL_SYMBOLS_OBJECT_VARIABLES_LOCATION_CARDINALITY_HPP
#define TEMPL_SYMBOLS_OBJECT_VARIABLES_LOCATION_CARDINALITY_HPP

#include <templ/symbols/constants/Location.hpp>

namespace templ {
namespace symbols {
namespace object_variables {

class LocationCardinality : public ObjectVariable
{
    constants::Location::Ptr mLocation;
    uint32_t mCardinality;
    owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType mCardinalityRestrictionType;

public:
    typedef boost::shared_ptr<LocationCardinality> Ptr;

    LocationCardinality(const constants::Location::Ptr location,
            uint32_t cardinality = 1,
            owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type = owlapi::model::OWLCardinalityRestriction::MIN
    )
        : ObjectVariable("unknown", LOCATION_CARDINALITY)
        , mLocation(location)
        , mCardinality(cardinality)
        , mCardinalityRestrictionType(type)
    {}

    virtual ~LocationCardinality() {}

    constants::Location::Ptr getLocation() const  { return mLocation; }
    uint32_t getCardinality() const { return mCardinality; }
    owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType getCardinalityRestrictionType() const { return mCardinalityRestrictionType; }

    virtual bool equals(const Symbol::Ptr& symbol) const
    {
        LocationCardinality::Ptr other = boost::dynamic_pointer_cast<LocationCardinality>(symbol);
        if(other)
        {
            return mLocation == other->mLocation && 
                mCardinality == other->mCardinality && 
                mCardinalityRestrictionType == other->mCardinalityRestrictionType;
        }
        return false;
    }
};

} // end namespace object_variables
} // end namespace symbols
} // end templ
#endif // TEMPL_SYMBOLS_OBJECT_VARIABLES_LOCATION_CARDINALITY_HPP
