#ifndef TEMPL_OBJECT_VARIABLES_LOCATION_CARDINALITY_HPP
#define TEMPL_OBJECT_VARIABLES_LOCATION_CARDINALITY_HPP

#include <templ/ObjectVariable.hpp>

namespace templ {
namespace object_variables {

class LocationCardinality : public ObjectVariable
{
    uint32_t mCardinality;
    owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType mCardinalityRestrictionType;

public:
    typedef boost::shared_ptr<LocationCardinality> Ptr;

    LocationCardinality(const std::string& location,
            uint32_t cardinality = 1,
            owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType type = owlapi::model::OWLCardinalityRestriction::MIN
    )
        : ObjectVariable(location, "Location-Cardinality")
        , mCardinality(cardinality)
        , mCardinalityRestrictionType(type)
    {}

    virtual ~LocationCardinality() {}

    const std::string& getLocation() const  { return getInstanceName(); }
    uint32_t getCardinality() const { return mCardinality; }
    owlapi::model::OWLCardinalityRestriction::CardinalityRestrictionType getCardinalityRestrictionType() const { return mCardinalityRestrictionType; }
};

} // end namespace object_variables
} // end templ
#endif // TEMPL_OBJECT_VARIABLES_LOCATION_CARDINALITY_HPP
