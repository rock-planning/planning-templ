#include <boost/test/unit_test.hpp>
#include <templ/Role.hpp>
#include <templ/RoleInfoWeightedEdge.hpp>
#include <templ/RoleInfoWeightedEdge.hpp>
#include <templ/symbols/constants/Location.hpp>
#include <templ/solvers/temporal/point_algebra/TimePoint.hpp>
#include <templ/RoleInfoTuple.hpp>

#include <boost/archive/text_oarchive.hpp>


#include <graph_analysis/BaseGraph.hpp>
#include <graph_analysis/GraphIO.hpp>
#include <graph_analysis/EdgeTypeManager.hpp>

using namespace templ;

BOOST_AUTO_TEST_SUITE(io)

BOOST_AUTO_TEST_CASE(role_serialization)
{
    Role role("test_role", "http://model/instance#0");

    std::stringstream ss;
    //boost::archive::text_oarchive oarch(ss);
    //oarch << role;
    BOOST_TEST_MESSAGE("Role serialized to: " << ss.str());
}

BOOST_AUTO_TEST_CASE(role_info_serialization)
{
    RoleInfo roleInfo;
    Role role("test_role", "http://model/instance#0");
    roleInfo.addRole(role);
    roleInfo.addRole(role,"tag-0");
    roleInfo.addRole(role,"tag-1");

    //BOOST_TEST_MESSAGE("RoleInfo serialized to: " << roleInfo.serializeRoles());
    //BOOST_TEST_MESSAGE("RoleInfo serialized to: " << roleInfo.serializeTaggedRoles());
}

BOOST_AUTO_TEST_CASE(role_info_weighted_edge_serialization_0)
{
    RoleInfoWeightedEdge e;

    for(size_t i = 0; i < 5; ++i)
    {
        std::stringstream ss;
        ss << "test_role-" << i;
        Role role(ss.str(), "http://model/instance#0");
        e.addRole(role);
    }

    using namespace graph_analysis;

    std::string filename = "/tmp/templ-test_io-role_info_weighted_edge_serialization.gexf";
    RoleInfoWeightedEdge original;
    {
        namespace con = templ::symbols::constants;
        namespace pa = templ::solvers::temporal::point_algebra;

        typedef RoleInfoTuple<con::Location::Ptr, pa::TimePoint::Ptr> RoleInfoSpaceTimeTuple;

        con::Location::Ptr l0 = con::Location::create("l0");
        con::Location::Ptr l1 = con::Location::create("l1");

        pa::TimePoint::Ptr t0 = pa::TimePoint::create("t0");
        pa::TimePoint::Ptr t1 = pa::TimePoint::create("t1");

        RoleInfoSpaceTimeTuple::Ptr v0(new RoleInfoSpaceTimeTuple(l0,t0));
        RoleInfoSpaceTimeTuple::Ptr v1(new RoleInfoSpaceTimeTuple(l1,t1));

        //Vertex::Ptr v0(new Vertex("v0"));
        //Vertex::Ptr v1(new Vertex("v1"));

        RoleInfoWeightedEdge::Ptr edge(new RoleInfoWeightedEdge(e));
        edge->setSourceVertex(v0);
        edge->setTargetVertex(v1);

        BaseGraph::Ptr graph = BaseGraph::getInstance();
        graph->addEdge(edge);
        original = *edge.get();

        graph_analysis::io::GraphIO::write(filename, graph, representation::GEXF);
    }

    {
        BaseGraph::Ptr read_graph = BaseGraph::getInstance();
        graph_analysis::io::GraphIO::read(filename, read_graph, representation::GEXF);

        EdgeIterator::Ptr edgeIt = read_graph->getEdgeIterator();
        while(edgeIt->next())
        {
            RoleInfoWeightedEdge::Ptr roleInfoEdge = dynamic_pointer_cast<RoleInfoWeightedEdge>(edgeIt->current());
            std::set<Role> roles = roleInfoEdge->getRoles();

            BOOST_REQUIRE_MESSAGE(roleInfoEdge->getRoles() == original.getRoles(), "Reloaded edge same roles as original");
            BOOST_REQUIRE_MESSAGE(roles.size() == 5, "RoleInfoEdge expected 5 Roles, but has " << roles.size());
            BOOST_TEST_MESSAGE("RoleInfoEdge: " << roleInfoEdge->toString());

            BOOST_REQUIRE_MESSAGE(roleInfoEdge->getSourceVertex(), "Source vertex is not null");
            BOOST_REQUIRE_MESSAGE(roleInfoEdge->getTargetVertex(), "Target vertex is not null");

            BOOST_TEST_MESSAGE("Source vertex: " << roleInfoEdge->getSourceVertex()->toString());
            BOOST_TEST_MESSAGE("Target vertex: " << roleInfoEdge->getTargetVertex()->toString());
        }
    }
}

BOOST_AUTO_TEST_CASE(role_info_weighted_edge_serialization_1)
{
    using namespace graph_analysis;
    EdgeTypeManager* eManager = EdgeTypeManager::getInstance();

    RoleInfoWeightedEdge::Ptr edge( new RoleInfoWeightedEdge());
    for(size_t i = 0; i < 5; ++i)
    {
        std::stringstream ss;
        ss << "test_role-" << i;
        Role role(ss.str(), "http://model/instance#0");
        edge->addRole(role);
    }
//    //eManager->registerType("RoleInfoWeightedEdge", edge, true);
//    //eManager->registerAttribute("RoleInfoWeightedEdge", "weights",
//    //           (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoWeightedEdge::serializeWeights,
//    //           (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoWeightedEdge::deserializeWeights,
//    //           (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&RoleInfoWeightedEdge::serializeWeights);
//    //eManager->registerAttribute("RoleInfoWeightedEdge", "roles",
//    //           (graph_analysis::io::AttributeSerializationCallbacks::serialize_func_t)&RoleInfoWeightedEdge::serializeRoles,
//    //           (graph_analysis::io::AttributeSerializationCallbacks::deserialize_func_t)&RoleInfoWeightedEdge::deserializeRoles,
//    //           (graph_analysis::io::AttributeSerializationCallbacks::print_func_t)&RoleInfoWeightedEdge::serializeRoles);
//
//
//
    std::vector<std::string> attributes = eManager->getAttributes(edge->getClassName());
    for(const std::string& attribute : attributes)
    {
        graph_analysis::io::AttributeSerializationCallbacks callbacks = eManager->getAttributeSerializationCallbacks(edge->getClassName(), attribute);
        std::set<Role> roles = edge->getRoles();
        BOOST_REQUIRE_MESSAGE(roles.size() == 5, "Number of roles " << roles.size());
        std::string s = (edge.get()->*callbacks.serializeFunction)();
        BOOST_TEST_MESSAGE("Serialized function: " << s);
        break;
    }
}

BOOST_AUTO_TEST_CASE(role_info_weighted_edge_serialization_2)
{

    RoleInfoWeightedEdge e;
    BOOST_TEST_MESSAGE("RoleInfoWeightedEdge roles serialized to: " << e.serializeRoles());
    BOOST_TEST_MESSAGE("RoleInfoWeightedEdge taggedRoles serialized to: " << e.serializeTaggedRoles());

    for(size_t i = 0; i < 5; ++i)
    {
        std::stringstream ss;
        ss << "test_role-" << i;
        Role role(ss.str(), "http://model/instance#0");
        e.addRole(role);
    }

    BOOST_TEST_MESSAGE("RoleInfoWeightedEdge roles serialized to: " << e.serializeRoles());
    BOOST_TEST_MESSAGE("RoleInfoWeightedEdge taggedRoles serialized to: " << e.serializeTaggedRoles());

    RoleInfoWeightedEdge deserialized;
    std::string s = e.serializeRoles();
    deserialized.deserializeRoles(s);
    //BOOST_REQUIRE_MESSAGE(deserialized.getRoles() == e.getRoles(), "Role are the same");

    //deserialized.deserializeTaggedRoles(e.serializeTaggedRoles());
    //BOOST_REQUIRE_MESSAGE(deserialized.getAllRoles() == e.getAllRoles(), "TaggedRole are the same");
}

BOOST_AUTO_TEST_SUITE_END()
