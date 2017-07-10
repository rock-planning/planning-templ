#include <boost/test/unit_test.hpp>
#include <sstream>
#include <templ/RoleInfo.hpp>

using namespace templ;

BOOST_AUTO_TEST_SUITE(role_info)

BOOST_AUTO_TEST_CASE(relative_complement)
{
    std::vector<Role> roles;
    for(size_t i = 0; i < 10; ++i)
    {
        std::stringstream ss;
        ss << "test_role_" << i;
        Role role(ss.str(), "http://model/instance#0");
        roles.push_back(role);
    }

    {
        RoleInfo a;
        a.addRole(roles[0]);
        a.addRole(roles[1]);
        a.addRole(roles[4]);

        a.addRole(roles[0], "assigned");
        a.addRole(roles[1], "assigned");
        a.addRole(roles[2], "assigned");
        a.addRole(roles[4], "assigned");
        a.addRole(roles[5], "assigned");

        {
            // all item are found in the other
            Role::List delta = a.getRelativeComplement("","assigned");
            BOOST_REQUIRE_MESSAGE(delta.size() == 0, "Delta should be 0");
        }

        {
            Role::List delta = a.getRelativeComplement("assigned","");
            BOOST_REQUIRE_MESSAGE(delta.size() == 2, "Delta should be 2: expected roles are 2 and 5 was " << Role::toString(delta));
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
