#ifndef TEMPL_CONSTRAINT_HPP
#define TEMPL_CONSTRAINT_HPP

#include "Variable.hpp"
#include <graph_analysis/Edge.hpp>

#define T_CONSTRAINT(x) dynamic_pointer_cast<templ::Constraint>(x)

namespace templ {

/**
 * A Constraint represents an edge in the constraint network
 *
 * Some implementation out of the existing constraint categories:
 * \see constraints::ModelConstraint
 * \see solvers::temporal::point_algebra::QualitativeTimePoint
 * \see solvers::temporal::IntervalConstraint
 */
class Constraint
{
public:
    typedef shared_ptr<Constraint> Ptr;
    typedef std::vector<Ptr> PtrList;

    enum Tag
    {
        UNKNOWN_TAG,
        /// e.g. when constraint originates from a mission definition
        PRIORITY_HIGH,
        /// e.g, when constraint originates from narrowing for a solution
        PRIORITY_LOW,
        END_TAG
    };

    static std::map<Tag, std::string> TagTxt;

    // A general category of the constraint
    enum Category
    {
        UNKNOWN,
        TEMPORAL_ASSERTION,
        TEMPORAL_QUALITATIVE,
        TEMPORAL_QUANTITATIVE,
        MODEL,
        END
    };

    static std::map<Category, std::string> CategoryTxt;


    /**
     * Default constructor for constraint
     */
    Constraint();

    /**
     * Constructor for a constraint using the category
     */
    Constraint(Category category);

    /**
     * Deconstructor
     */
    virtual ~Constraint();

    virtual bool operator==(const Constraint& other) const;

    bool operator!=(const Constraint& other) const { return !(*this == other); }

    std::set<Tag> getTags() const { return mTags; }

    /**
     * Add a tag to this constraint
     */
    void addTag(const Tag& tag);

    /**
     * Test if constraint has a particular tag
     */
    bool hasTag(const Tag& tag) const;

    /**
     * Get the category of this constraint
     * \return category
     */
    Category getCategory() const { return mCategory; }

    /**
     * Get the class name of this constraint
     * \return classname
     */
    virtual std::string getClassName() const = 0;

    /**
     * Get stringified object
     * \return string representation
     */
    virtual std::string toString() const { return toString(0); }

    /**
     * Get stringified object with indentation
     * \return string representation
     */
    virtual std::string toString(uint32_t indent) const = 0;

    /**
     * Get string object for list of constraints
     * \return string object
     */
    static std::string toString(const PtrList& constraints, size_t indent);

    /**
     * Get string from tags
     */
    std::string getTagsAsString() const;

private:
    Category mCategory;
    std::set<Tag> mTags;
};

typedef std::vector<Constraint::Ptr> ConstraintList;

} // end namespace templ
#endif // TEMPL_CONSTRAINT_HPP
