#ifndef TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP

#include <graph_analysis/EdgeRegistration.hpp>
#include "QualitativeTimePoint.hpp"
#include "../../../constraints/SimpleConstraint.hpp"

namespace templ {
namespace solvers {
namespace temporal {
namespace point_algebra {

/**
 * A qualitative timepoint constraint does not take into account bounds of any
 * timepoint.
 * Thus one should use instances of QualitativeTimePoint when working with this
 * kind of constraint
 *
 * Automated Planning p 290 Chap 13.3.1 Point Algebra
 */
class QualitativeTimePointConstraint : public constraints::SimpleConstraint
{
    static const graph_analysis::EdgeRegistration<QualitativeTimePointConstraint> __attribute__((used)) msRegistration;
public:
    /// Primitive relations are P = {<,=,>} and timepoints can be related in only
    /// these three ways
    /// Constraining can be done in additional ways, i.e. <=, >=, !=
    ///
    /// Here we rely on the qualititive constraints
    /// R =2^P = { {}{<},{=},{>},{<,=},{>,=},{<,>},P}
    ///
    enum Type { Empty, Greater, Less, Equal, Distinct, GreaterOrEqual, LessOrEqual, Universal, TypeEndMarker };

    typedef shared_ptr<QualitativeTimePointConstraint> Ptr;

    /**
     * Default constructor
     */
    QualitativeTimePointConstraint();

    QualitativeTimePointConstraint(const Variable::Ptr& source,
            const Variable::Ptr& target,
            Type constraintType);

    bool operator==(const QualitativeTimePointConstraint& other) const;

    void setLabel(const std::string& label) override;

    std::string getClassName() const override { return "QualitativeTimePointConstraint"; }

    std::string toString(uint32_t indent) const override;

    void setType(QualitativeTimePointConstraint::Type type);

    Type getType() const { return mConstraintType; }

    static QualitativeTimePointConstraint::Ptr create(const Variable::Ptr& source,
            const Variable::Ptr& target,
            Type constraintType);

    /**
     * Get referenced timepoint interpreted as lvalue (left hand value) of the corresponding
     * constraint
     */
    TimePoint::Ptr getLVal() const { return dynamic_pointer_cast<TimePoint>( getSourceVariable() ); }

    /**
     * Get referenced timepoint interpreted as rvalue (right hand value) of the corresponding
     * constraint
     */
    TimePoint::Ptr getRVal() const { return dynamic_pointer_cast<TimePoint>( getTargetVariable() ); }

    static QualitativeTimePointConstraint::Ptr getSymmetric(const QualitativeTimePointConstraint::Ptr& constraint);

    /**
     * Get the symmetric constraint
     * \return symmetric constraint
     */
    static Type getSymmetric(Type type);

    /**
     * Get the composition of two types in order to handle transitivity
     * \return composition type
     * \throws std::runtime_error if no composition type could be found
     */
    static Type getComposition(Type firstType, Type secondType);

    /**
     * Get the composition of a list of types
     * \param typeList List of composition types
     * \return composition type when all types in the list are consistent
     * \throws std::runtime_error if no composition type could be found
     */
    static Type getComposition(const std::vector<Type>& typeList);

    /**
     * Get the intersection of two types
     * \return intersection
     */
    static Type getIntersection(Type firstType, Type secondType);

    /**
     * Check if the two types have an intersection
     */
    static bool hasIntersection(Type firstType, Type secondType);

    /**
     * Check if a constraint type is consistent with another
     */
    static bool isConsistent(Type firstType, Type secondType);

    /**
     * Retrieve a list that contains all available constraint types
     * \return constraint type list
     */
    static std::vector<Type> getAllConstraintTypes();

    /// Symmetric constraint: change the primitive '<' by '>' and vice versa
    static std::map<Type, Type> SymmetricType;
    static std::map<Type, std::string> TypeTxt;

    /// Composition table for the time-point algebra for '<','=','>' and the
    /// additional '<=','>='
    ///
    /// \see "Automated Planning - Theory and Practice" (Chap. 13.3 Qualitative
    ///       Temporal Relations)
    ///
    ///  o| < | = | >
    ///  <| < | < | P
    ///  =| < | = | >
    ///  >| P | > | >
    ///
    /// The operator o is associative and distributive, i.e.
    /// {<,=} o {>,=} = {<}o{>} \cup {<}o{=} \cup {=}o{>} \cup {=}o{>} = {P} \cup
    /// {>} \cup {<} \cup {>} = {P}
    static std::map< std::pair<Type,Type>, Type > TypeAlgebra;

    static bool msInitializedCompositionTable;
    static bool msInitializedSymmetryTable;
protected:
    virtual graph_analysis::Edge* getClone() const { return new QualitativeTimePointConstraint(*this); }

private:
    Type mConstraintType;

    static QualitativeTimePointConstraint::Type msCompositionTable[8][8];
    static QualitativeTimePointConstraint::Type msSymmetryTable[8];
};

} // end namespace point_algebra
} // end namespace temporal
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_SOLVERS_TEMPORAL_TIMEPOINT_CONSTRAINT_HPP
