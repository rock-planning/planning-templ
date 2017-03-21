#include <boost/test/unit_test.hpp>
#include <templ/solvers/agent_routing/AgentRoutingProblem.hpp>
#include <templ/solvers/agent_routing/ArpIO.hpp>
#include "../test_utils.hpp"

using namespace templ;
using namespace templ::solvers::agent_routing;

struct ARPFixture
{
    AgentRoutingProblem arp;

    ARPFixture()
    {
        AgentType agentType(0);
        AgentIntegerAttribute type0_mobile(0, "mobile");
        type0_mobile.setValue(1);
        AgentIntegerAttribute type0_capacity(1, "transport-capacity");
        type0_capacity.setValue(100);
        AgentIntegerAttribute type0_costFactor(2, "transport-cost-factor");
        type0_costFactor.setValue(21);

        agentType.addIntegerAttribute(type0_mobile);
        agentType.addIntegerAttribute(type0_capacity);
        agentType.addIntegerAttribute(type0_costFactor);

        arp.addAgentType(agentType);

        Agent agent;
        agent.setAgentType(0);
        agent.setAgentId(11);

        AgentIntegerAttribute mobile(0, "mobile");
        AgentIntegerAttribute capacity(1, "transport-capacity");
        AgentIntegerAttribute costFactor(2, "transport-cost-factor");

        arp.addIntegerAttribute(mobile);
        arp.addIntegerAttribute(capacity);
        arp.addIntegerAttribute(costFactor);

        AgentTask task;
        task.setTaskPriority(99);
        task.setLocation( symbols::constants::Location("loc-0") );
        task.setTaskDuration(10);
        using namespace solvers::temporal::point_algebra;
        TimePoint::Ptr t0(new QualitativeTimePoint("t0"));
        TimePoint::Ptr t1(new QualitativeTimePoint("t1"));
        task.setArrival(t0);
        task.setDeparture(t1);

        agent.addTask(task);

        for(uint32_t i = 1; i < 10; ++i)
        {
            arp.addAgentType(i);
        }

        arp.addAgent(agent);
    }
};

BOOST_AUTO_TEST_SUITE(agent_routing)

BOOST_AUTO_TEST_CASE(xml_writer)
{
    ARPFixture fixture;

    std::string path = "/tmp/templ-test-agent_routing-xml_writer.xml";
    ArpIO::write(path, fixture.arp, representation::XML);
    BOOST_TEST_MESSAGE("Writing file");
};


BOOST_AUTO_TEST_CASE(xml_reader)
{
    std::string path = getRootDir() + "test/data/agent_routing/agent_routing-xml_reader.xml";
    AgentRoutingProblem arp;
    ArpIO::read(path, arp, representation::XML);
    BOOST_TEST_MESSAGE("Reading file");
}
BOOST_AUTO_TEST_SUITE_END()
