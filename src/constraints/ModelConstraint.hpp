#ifndef TEMPL_CONSTRAINTS_MODEL_CONSTRAINT_HPP
#define TEMPL_CONSTRAINTS_MODEL_CONSTRAINT_HPP

#include <map>
#include <map>
#include <owlapi/model/IRI.hpp>
#include "../SpaceTime.hpp"
#include "HyperConstraint.hpp"

namespace templ {
namespace constraints {

/**
 * Define the set of general mission constraints which can be used to details
 * the planning problem
 */
class ModelConstraint : public constraints::HyperConstraint
{
public:
    typedef std::vector<ModelConstraint> List;
    typedef shared_ptr<ModelConstraint> Ptr;

    enum Type { UNKNOWN,
        MIN_DISTINCT, MAX_DISTINCT, ALL_DISTINCT,
        MIN_EQUAL, MAX_EQUAL,
        MIN_FUNCTION, MAX_FUNCTION,
        MIN_PROPERTY, MAX_PROPERTY,
        END_TYPE };

    static std::map<Type, std::string> TypeTxt;

    ModelConstraint();

    ModelConstraint(Type type,
            const owlapi::model::IRI& model,
            const std::vector<SpaceTime::SpaceIntervalTuple>& affectedSpaceIntervals,
            uint32_t value = 0,
            const owlapi::model::IRI& property = owlapi::model::IRI());

    std::string getClassName() const override { return "ModelConstraint"; }

    std::string toString() const override { return toString(0); }

    std::string toString(uint32_t indent) const override;

    const owlapi::model::IRI& getModel() const { return mModel; }
    uint32_t getValue() const { return mValue; }
    const owlapi::model::IRI& getProperty() const { return mProperty; }
    const std::vector<SpaceTime::SpaceIntervalTuple>& getSpaceIntervalTuples() const { return mSpaceIntervalTuples; }

    Type getModelConstraintType() const { return mType; }

    static Type getTypeFromTxt(const std::string& txt);


protected:
    Type mType;
    owlapi::model::IRI mModel;
    /// Represent the space-time interval to which this constraint applies
    std::vector<SpaceTime::SpaceIntervalTuple> mSpaceIntervalTuples;
    uint32_t mValue;
    owlapi::model::IRI mProperty;

    graph_analysis::Vertex* getClone() const override { return new ModelConstraint(*this); }
};

} // end namespace constraints
} // end namespace templ
#endif //TEMPL_CONSTRAINTS_MODEL_CONSTRAINT_HPP
