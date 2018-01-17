#ifndef TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP
#define TEMPL_SOLVERS_TEMPORAL_QUALITATIVE_TIMEPOINT_CONSTRAINT_HPP

#include <templ/constraints/SimpleConstraint.hpp>
#include <templ/solvers/temporal/point_algebra/QualitativeTimePoint.hpp>

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

    virtual void setLabel(const std::string& label);

    virtual std::string getClassName() const { return "QualitativeTimePointConstraint"; }

    virtual std::string toString() const;

    void setType(QualitativeTimePointConstraint::Type type);

    Type getType() const { return mConstraintType; }

    static QualitativeTimePointConstraint::Ptr create(Variable::Ptr source, Variable::Ptr target, Type constraintType);

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
