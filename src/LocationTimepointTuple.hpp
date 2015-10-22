#ifndef TEMPL_LOCATION_TIMEPOINT_TUPLE_HPP
#define TEMPL_LOCATION_TIMEPOINT_TUPLE_HPP

#include <set>
#include <templ/Role.hpp>
#include <templ/Tuple.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>

namespace templ {

class LocationTimepointTuple : public Tuple<templ::symbols::constants::Location, templ::solvers::temporal::point_algebra::TimePoint>
{
public:
    typedef boost::shared_ptr<LocationTimepointTuple> Ptr;
    typedef Tuple<templ::symbols::constants::Location, templ::solvers::temporal::point_algebra::TimePoint> BaseClass;

    LocationTimepointTuple(const templ::symbols::constants::Location::Ptr& location,
            const templ::solvers::temporal::point_algebra::TimePoint::Ptr& timepoint)
        : BaseClass(location, timepoint)
    {}

    void addRole(const Role& role) { mRoles.insert(role); }

    const std::set<Role>& getRoles() const { return mRoles; }
    std::string getClassName() const { return "LocationTimepointTuple"; }

    std::string toString() const
    {
        std::stringstream ss;
        ss << first()->toString() + "-" << second()->toString() << std::endl;
        ss << "    roles:" << std::endl;
        std::set<Role>::const_iterator it = mRoles.begin();
        for(; it != mRoles.end(); ++it)
        {
            ss << "        " << it->toString() << std::endl;
        }
        return ss.str();
    }

protected:
    Vertex* getClone() const { return new LocationTimepointTuple(*this); }

private:
    std::set<Role> mRoles;
};

} // end namespace templ
#endif // TEMPL_LOCATION_TIMEPOINT_TUPLE_HPP
