#ifndef TEMPL_MISSION_CONSTRAINT_HPP
#define TEMPL_MISSION_CONSTRAINT_HPP

#include <map>
#include <map>
#include <owlapi/model/IRI.hpp>
#include "SpaceTime.hpp"

namespace templ {
/**
 * Define the set of general mission constraints which can be used to details
 * the planning problem
 */
class MissionConstraint
{
public:
    typedef std::vector<MissionConstraint> List;

    enum Type { UNKNOWN, MIN_DISTINCT, MAX_DISTINCT, ALL_DISTINCT, MIN_EQUAL, MAX_EQUAL, MIN_FUNCTION, MAX_FUNCTION, END_TYPE };

    static std::map<Type, std::string> TypeTxt;


    MissionConstraint();

    MissionConstraint(Type type,
            const owlapi::model::IRI& model,
            const std::vector<SpaceTime::SpaceIntervalTuple>& affectedSpaceIntervals,
            uint32_t value = 0,
            const owlapi::model::IRI& property = owlapi::model::IRI());

    const owlapi::model::IRI& getModel() const { return mModel; }
    uint32_t getValue() const { return mValue; }
    const owlapi::model::IRI& getProperty() const { return mProperty; }
    const std::vector<SpaceTime::SpaceIntervalTuple>& getSpaceIntervalTuples() const { return mSpaceIntervalTuples; }

    std::string toString(size_t indent = 0) const;

    static Type getTypeFromTxt(const std::string& txt);


protected:
    Type mType;
    owlapi::model::IRI mModel;
    /// Represent the space-time interval to which this constraint applies
    std::vector<SpaceTime::SpaceIntervalTuple> mSpaceIntervalTuples;
    uint32_t mValue;
    owlapi::model::IRI mProperty;
};
} // end namespace templ
#endif //TEMPL_MISSION_CONSTRAINT_HPP
