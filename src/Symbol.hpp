#ifndef TEMPL_SYMBOL_HPP
#define TEMPL_SYMBOL_HPP

#include <map>
#include <string>
#include <utility>
#include <templ/SharedPtr.hpp>

namespace templ {

class Symbol : public std::pair<std::string, std::string>
{
public:
    typedef shared_ptr<Symbol> Ptr;

    enum Type { UNKNOWN, STATE_VARIABLE, OBJECT_VARIABLE, CONSTANT, TEMPORAL_VARIABLE, VALUE };
    static std::map<Type, std::string> TypeTxt;

    Symbol(const std::string& name, const std::string& type_name, Type type);

    virtual ~Symbol() {}

    const std::string& getInstanceName() const { return first; }
    const std::string& getTypeName() const { return second; }

    Type getType() const { return mType; }

    virtual bool equals(const Symbol::Ptr& other) const { return first == other->first && second == other->second && mType == other->mType; }

    virtual std::string toString() const;

private:
    Type mType;
};

} // end namespace templ
#endif // TEMPL_SYMBOL_HPP
