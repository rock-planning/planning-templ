#include "Constraint.hpp"
#include <iostream>

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
    { Constraint::TEMPORAL_QUANTIATIVE, "TEMPORAL_QUANTIATIVE" },
    { Constraint::MODEL,    "MODEL" }
};

} // end namespace templ
