#include <templ/io/MissionReader.hpp>
//#include <templ/MissionPlanner.hpp>

#include <templ/solvers/csp/RoleDistribution.hpp>

using namespace templ;

class RoleTimeline
{
    std::vector<solvers::csp::FluentTimeResource> mFluents;
    Role mRole;

public:
    void setRole(const Role& role) { mRole = role; }

    void add(const solvers::csp::FluentTimeResource& fts) { mFluents.push_back(fts); }

    void sort(const std::vector<solvers::temporal::Interval>& intervals)
    {
        using namespace solvers::csp;
        std::sort( mFluents.begin(), mFluents.end(), [&intervals](const FluentTimeResource& a, const FluentTimeResource& b)->bool
                {
                    if(a == b)
                    {
                        return false;
                    }

                    const solvers::temporal::Interval& lval = intervals.at(a.time);
                    const solvers::temporal::Interval& rval = intervals.at(b.time);
                    return lval.before(rval);
                });
    }

    std::string toString() const
    {
        std::stringstream ss;
        ss << "-- Timeline: " << mRole.toString() << std::endl;
        for(auto &fluent : mFluents)
        {
            ss << fluent.toString() << std::endl;
        }
        return ss.str();
    }

};

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        printf("usage: %s <mission> <organization-model>\n", argv[0]);
        exit(-1);
    }

    std::string missionFilename = argv[1];
    std::string organizationModelFilename = argv[2];

    using namespace templ;
    Mission mission = io::MissionReader::fromFile(missionFilename);
    printf("%s\n",mission.toString().c_str());

    using namespace organization_model;
    OrganizationModel::Ptr organizationModel = OrganizationModel::getInstance(organizationModelFilename);
    mission.setOrganizationModel(organizationModel);

    mission.prepare();

    std::vector<solvers::csp::ModelDistribution::Solution> solutions = solvers::csp::ModelDistribution::solve(mission);
    if(!solutions.empty())
    {
        std::cout << "Solutions for model distribution found: " << solutions << std::endl;
    } else {
        throw std::runtime_error("No feasible model distribution");
    }

    // Check role distribution after model distribution, i.e., using one
    // solution of the model distribution to find a role assignment
    std::vector<solvers::csp::RoleDistribution::Solution> roleSolutions = solvers::csp::RoleDistribution::solve(mission, solutions[0]);
    if(!roleSolutions.empty())
    {
        std::cout << "Solutions for role distribution: " << roleSolutions << std::endl;
    } else {
        throw std::runtime_error("No feasible role distribution");
    }


    std::cout << "-- BEGIN TIMELINES --------" << std::endl;

    // Compute the timelines per role for one role solution
    std::map<Role, RoleTimeline> timelines;

    {
        using namespace solvers::csp;

        // Timeline
        RoleDistribution::Solution roleSolution = roleSolutions[0];
        RoleDistribution::Solution::const_iterator cit = roleSolution.begin();
        for(; cit != roleSolution.end(); ++cit)
        {
            const Role::List& roles = cit->second;
            const FluentTimeResource& fts = cit->first;

            Role::List::const_iterator lit = roles.begin();
            for(; lit != roles.end(); ++lit)
            {
                const Role& role = *lit;
                timelines[role].setRole(role);
                timelines[role].add(fts);
            }
        }

        // Sort timeline
        std::vector<solvers::temporal::Interval> intervals(mission.getTimeIntervals().begin(), mission.getTimeIntervals().end());

        std::map<Role, RoleTimeline>::iterator it = timelines.begin();
        for(; it != timelines.end(); ++it)
        {
            RoleTimeline& timeline = it->second;
            timeline.sort(intervals);

            std::cout << timeline.toString() << std::endl;
        }
    }
    

    // Analyse the cost of the planning approach



//    MissionPlanner missionPlanner(mission);
//    CandidateMissions missions = missionPlanner.solve();

    return 0;
}
