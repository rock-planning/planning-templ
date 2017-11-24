#ifndef TEMPL_CONSTRAINT_HPP
#define TEMPL_CONSTRAINT_HPP

#include "Variable.hpp"
#include <graph_analysis/Edge.hpp>

#define T_CONSTRAINT(x) dynamic_pointer_cast<templ::Constraint>(x)

namespace templ {

/**
 * A Constraint represents an edge in the constraint network
 */
class Constraint
{
public:
    typedef shared_ptr<Constraint> Ptr;
    typedef std::vector<Ptr> PtrList;

    enum Category { UNKNOWN,
        TEMPORAL_QUALITATIVE,
        TEMPORAL_QUANTITATIVE,
        MODEL
    };

    static std::map<Category, std::string> CategoryTxt;

    Constraint();

    /**
     * Default constructor for a constraint
     */
    Constraint(Category category);

    /**
     * Deconstructor
     */
    virtual ~Constraint();

    /**
     * Get the type of this constraint
     */
    Category getCategory() const { return mCategory; }

    /**
     * Get the class name of this constraint
     * \return classname
     */
    virtual std::string getClassName() const = 0;

    /**
     * Get stringified object
     * \return string repr
     */
    virtual std::string toString() const { return toString(0); }

    virtual std::string toString(uint32_t indent) const = 0;

private:
    Category mCategory;
};

typedef std::vector<Constraint::Ptr> ConstraintList;

} // end namespace templ
#endif // TEMPL_CONSTRAINT_HPP
