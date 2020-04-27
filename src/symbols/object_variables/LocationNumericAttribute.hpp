#ifndef TEMPL_SYMBOLS_OBJECT_VARIABLES_LOCATION_NUMERIC_ATTRIBUTE_HPP
#define TEMPL_SYMBOLS_OBJECT_VARIABLES_LOCATION_NUMERIC_ATTRIBUTE_HPP

#include "../constants/Location.hpp"
#include "../ObjectVariable.hpp"
#include <owlapi/OWLApi.hpp>

namespace templ {
namespace symbols {
namespace object_variables {

class LocationNumericAttribute : public ObjectVariable
{
    constants::Location::Ptr mLocation;
    owlapi::model::IRI mNumericAttribute;
    int32_t mMinInclusive;
    int32_t mMaxInclusive;

public:
    typedef shared_ptr<LocationNumericAttribute> Ptr;

    LocationNumericAttribute(const constants::Location::Ptr& location,
            const owlapi::model::IRI& mNumericAttribute,
            int32_t minInclusive,
            int32_t maxInclusive
    );

    virtual ~LocationNumericAttribute();

    constants::Location::Ptr getLocation() const  { return mLocation; }

    int32_t getMinInclusive() const { return mMinInclusive; }
    int32_t getMaxInclusive() const { return mMaxInclusive; }

    owlapi::model::IRI getNumericAttribute() const { return mNumericAttribute; }

    virtual bool equals(const Symbol::Ptr& symbol) const;
    virtual std::string toString() const;
};

} // end namespace object_variables
} // end namespace symbols
} // end templ
#endif // TEMPL_SYMBOLS_OBJECT_VARIABLES_LOCATION_FLOW_HPP
