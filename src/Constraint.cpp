#include "Constraint.hpp"
#include <iostream>

#include "constraints/ModelConstraint.hpp"
#include "constraints/SimpleConstraint.hpp"

using namespace graph_analysis;

namespace templ {
Constraint::Constraint()
    : mCategory(UNKNOWN)
{}

Constraint::Constraint(Category category)
    : mCategory(category)
{}

Constraint::~Constraint()
{}

std::map<Constraint::Category, std::string> Constraint::CategoryTxt = {
    { Constraint::UNKNOWN,  "UNKNOWN" },
    { Constraint::TEMPORAL_QUALITATIVE, "TEMPORAL_QUALITATIVE" },
    { Constraint::TEMPORAL_QUANTITATIVE, "TEMPORAL_QUANTITATIVE" },
    { Constraint::MODEL,    "MODEL" }
};

bool Constraint::operator==(const Constraint& other) const
{
    return mCategory == other.mCategory;
}

std::string Constraint::toString(const Constraint::PtrList& constraints, size_t indent)
{
    std::string hspace(indent,' ');
    std::stringstream ss;
    ss << hspace << "Constraints:" << std::endl;
    for(const Constraint::Ptr& constraint : constraints)
    {
        ss << constraint->toString(indent + 4);
    }
    return ss.str();
}

} // end namespace templ
