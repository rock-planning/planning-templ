#ifndef TEMPL_AGENT_ROUTING_AGENT_INTEGER_ATTRIBUTE_HPP
#define TEMPL_AGENT_ROUTING_AGENT_INTEGER_ATTRIBUTE_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace templ {
namespace solvers {
namespace agent_routing {

typedef std::string AttributeName;

class AgentIntegerAttribute
{
public:
    typedef std::vector<AgentIntegerAttribute> List;

    AgentIntegerAttribute();
    AgentIntegerAttribute(uint32_t id, const AttributeName& label = "");

    bool isValid() const;

    void setId(uint32_t id) { mId = id; }
    uint32_t getId() const { return mId; }

    void setLabel(const AttributeName& label) { mLabel = label; }
    const AttributeName& getLabel() const { return mLabel; }

    void setValue(uint32_t value) { mValue = value; }
    uint32_t getValue() const { return mValue; }

    void setMinValue(uint32_t value) { mMinValue = value; }
    uint32_t getMinValue() const { return mMinValue; }

    void setMaxValue(uint32_t value) { mMaxValue = value; }
    uint32_t getMaxValue() const { return mMaxValue; }

    std::string toString(uint32_t indent = 0) const;
    static std::string toString(const List& list, uint32_t indent = 0);

protected:
    uint32_t mId;
    AttributeName mLabel;

    uint32_t mMinValue;
    uint32_t mMaxValue;

    uint32_t mValue;
};

} // end namespace agent_routing
} // end namespace solvers
} // end namespace templ
#endif // TEMPL_AGENT_ROUTING_AGENT_INTEGER_ATTRIBUTE_HPP
