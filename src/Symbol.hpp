#ifndef TEMPL_SYMBOL_HPP
#define TEMPL_SYMBOL_HPP

#include <vector>
#include <map>
#include <string>
#include <utility>
#include <templ/SharedPtr.hpp>

namespace templ {

/**
 * \class (Typed) Symbol
 * \brief represent a general symbol with a given type
 *
 */
class Symbol : public std::pair<std::string, std::string>
{
public:
    typedef shared_ptr<Symbol> Ptr;
    typedef std::vector< Ptr > SymbolPtrList;

    enum Type { UNKNOWN, STATE_VARIABLE, OBJECT_VARIABLE, CONSTANT, TEMPORAL_VARIABLE, VALUE };
    static std::map<Type, std::string> TypeTxt;

    /**
     * \param name Name of the symbol
     * \param type_name Type name of the symbol
     * \param type Classtype of the symbol
     */
    Symbol(const std::string& name, const std::string& type_name, Type type);

    virtual ~Symbol() {}

    /**
     * Get the instance name of the symbool
     * \return instance name
     */
    const std::string& getInstanceName() const { return first; }

    /**
     * Get the typename of the symobl
     */
    const std::string& getTypeName() const { return second; }

    /**
     * Get the actual type of the symbol
     */
    Type getType() const { return mType; }

    virtual bool equals(const Symbol::Ptr& other) const { return first == other->first && second == other->second && mType == other->mType; }

    virtual std::string toString() const;
    virtual std::string toString(size_t indent) const;

private:
    Type mType;
};

} // end namespace templ
#endif // TEMPL_SYMBOL_HPP
