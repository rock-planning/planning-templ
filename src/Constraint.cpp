#include "Constraint.hpp"

using namespace graph_analysis;

namespace templ {

Constraint::Constraint(Type type)
    : mType(type)
{}

Constraint::~Constraint()
{}

std::map<Constraint::Type, std::string> Constraint::TypeTxt = {
    { Constraint::UNKNOWN,  "UNKNOWN" },
    { Constraint::TEMPORAL_QUALITATIVE, "TEMPORAL_QUALITATIVE" },
    { Constraint::TEMPORAL_QUANTIATIVE, "TEMPORAL_QUANTIATIVE" },
    { Constraint::MODEL,    "MODEL" }
};

} // end namespace templ
