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

std::map<Constraint::Tag, std::string> Constraint::TagTxt = {
    { Constraint::UNKNOWN_TAG, "UNKNOWN" },
    { Constraint::PRIORITY_HIGH, "PRIORITY_HIGH" },
    { Constraint::PRIORITY_LOW, "PRIORITY_LOW" },
    { Constraint::END_TAG, "END" }
};


std::map<Constraint::Category, std::string> Constraint::CategoryTxt = {
    { Constraint::UNKNOWN,  "UNKNOWN" },
    { Constraint::TEMPORAL_QUALITATIVE, "TEMPORAL_QUALITATIVE" },
    { Constraint::TEMPORAL_QUANTITATIVE, "TEMPORAL_QUANTITATIVE" },
    { Constraint::MODEL,    "MODEL" }
};

void Constraint::addTag(const Tag& tag)
{
    mTags.insert(tag);
}

bool Constraint::hasTag(const Tag& tag) const
{
    return mTags.end() != std::find(mTags.begin(), mTags.end(), tag);
}

bool Constraint::operator==(const Constraint& other) const
{
    return mCategory == other.mCategory && mTags == other.mTags;
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

std::string Constraint::getTagsAsString() const
{
    std::stringstream ss;
    ss << "[";
    std::vector<Tag> tags(mTags.begin(), mTags.end());
    for(size_t i = 0; i < tags.size(); ++i)
    {
        ss << TagTxt[ tags[i] ];
        if( i < tags.size()-1 )
        {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

} // end namespace templ
