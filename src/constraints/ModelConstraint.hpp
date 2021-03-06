#ifndef TEMPL_CONSTRAINTS_MODEL_CONSTRAINT_HPP
#define TEMPL_CONSTRAINTS_MODEL_CONSTRAINT_HPP

#include <map>
#include <map>
#include <owlapi/model/IRI.hpp>
#include <owlapi/model/OWLOntologyAsk.hpp>
#include "../SpaceTime.hpp"
#include "HyperConstraint.hpp"

namespace templ {
namespace constraints {

/**
 * Define the set of general mission constraints which can be used to detail
 * the planning problem
 * A typical constraint is a tuple of <M,R,P,V>, where
 * M is the model, R is the set of requirement ids, P is the property label and
 * V is the value that is used as reference value for this constraint
 * Property and value are optional
 *
 * \see io::MissionReader for details on how the XML encoding is done
 */
class ModelConstraint : public constraints::HyperConstraint
{
public:
    typedef std::vector<ModelConstraint> List;
    typedef shared_ptr<ModelConstraint> Ptr;

    /// Available model constraint types
    enum Type {
        /// Unknown constraint
        UNKNOWN,
        // Maximum usage of agent instances across multiple requirements
        MAX,
        // Minimum usage of agent instances across multiple requirements
        MIN,
        // Maximum entries allowed to a single location
        MAX_ACCESS,
        // Minimum entries required to a single location
        MIN_ACCESS,
        /// \f$ minDistinct(S,\widehat{a},n) \f$ describes
        /// the constraint: \f$ \forall s_i,s_j \in S, i \neq j: \left|
        /// |A_{s_i}^{\widehat{a}}| - |A_{s_j}^{\widehat{a}}| \right| \geq n \f$,
        /// where \f$ n > 0 \f$, \f$ Si \subset STR \f$, and \f$ A_s^{\widehat{a}}
        /// \f$ represents the filtered set of A which contains only agents of
        /// type \f$ \widehat{a} \f$ and is associated with the spatio-temporally
        /// qualified expression s.
        MIN_DISTINCT,
        /// \f$ maxDistinct(S,\widehat{a},n) \f$ describe the equivalent maximum
        /// constraint to \p minDistinct
        MAX_DISTINCT,
        /// \f$ allDistinct(STR,\widehat{a}) \f$ describes
        /// the constraint: \f$ \forall s \in STR: \bigcap A_s^{\widehat{a}} =
        /// \emptyset \f$ , where \f$ A_s^{\widehat{a}} \f$ represents the subset of
        /// agents of type \f$ \widehat{a} \f$ which are associated with the
        /// spatio-temporally qualified expression s.
        ALL_DISTINCT,
        MIN_EQUAL,
        MAX_EQUAL,
        ALL_EQUAL,
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

    ModelConstraint(Type type,
            const owlapi::model::IRI& model,
            const SpaceTime::SpaceIntervalTuple& affectedSpaceInterval,
            uint32_t value = 0,
            const owlapi::model::IRI& property = owlapi::model::IRI());

    bool operator==(const Constraint& other) const override;

    std::string getClassName() const override { return "ModelConstraint"; }

    std::string toString() const override { return toString(0); }

    std::string toString(uint32_t indent) const override;

    void setModel(const owlapi::model::IRI& model) { mModel = model; }

    /**
     * Get the refered model of the constraint
     * \return model
     */
    const owlapi::model::IRI& getModel() const { return mModel; }

    void setValue(uint32_t value) { mValue = value; }

    /**
     * Get the value of the constraint
     * \return value
     */
    uint32_t getValue() const { return mValue; }

    void setProperty(const owlapi::model::IRI& property) { mProperty = property; }

    /**
     * Get the property this model related to
     * \return property or owlapi::model::IRI() if the property should not be
     * used
     */
    const owlapi::model::IRI& getProperty() const { return mProperty; }

    void setSpaceIntervalTuples(const std::vector<SpaceTime::SpaceIntervalTuple>& tuples) { mSpaceIntervalTuples = tuples; }
    /**
     * Get the interval to which this constraint should be applied to
     * \return space time interval
     */
    const std::vector<SpaceTime::SpaceIntervalTuple>& getSpaceIntervalTuples() const { return mSpaceIntervalTuples; }

    void setModelConstraintType(Type type) { mType = type; }

    /**
     * Get the model constraint type
     * \return constraint type
     */
    Type getModelConstraintType() const { return mType; }

    /**
     * Get the type for a given string, e.g.
     * min-distinct, max-distinct, all-distinct,
     * min-equal, max-equal, min-function, max-function,
     * min-property, max-property
     *
     */
    static Type getTypeFromTxt(const std::string& txt);

    /**
     * Validate the model constraints values with respect to a given ontology
     */
    void validate(const owlapi::model::OWLOntologyAsk& ask) const;


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
