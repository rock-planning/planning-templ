#ifndef TEMPL_SYMBOLS_CONSTANT_HPP
#define TEMPL_SYMBOLS_CONSTANT_HPP

#include <vector>
#include <map>
#include "../Symbol.hpp"

namespace templ {
namespace symbols {

/**
 * \class Constant
 * \brief Constant symbols describe parts of the planning domain that remain
 * 'constant', e.g., locations or robots
 */
class Constant : public Symbol
{
public:
    typedef shared_ptr<Constant> Ptr;

    enum Type : size_t { UNKNOWN, LOCATION, END };
    static std::map<Type, std::string> TypeTxt;

    Constant(const std::string& name, Type type);
    virtual ~Constant() {}

    Type getConstantType() const { return mConstantType; }

private:
    Type mConstantType;
};

typedef std::vector<Constant::Ptr> ConstantList;

} // end namespace symbols
} // end namespace templ
#endif // TEMPL_SYMBOLS_CONSTANT_HPP
