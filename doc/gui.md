# Templ GUI

The program 'templ-gui' is the frontend to the planning and permits to trigger the planning and visualize and inspect resulting final and intermediate solutions.

## Editing a Mission

Mission editing can be done via:
File->New->MissionEditor

Save the resulting mission into an xml file.

## Import/View a Mission
File->Import->Import Mission

## Plan a Mission
You first have to import/view a mission and remain on the corresponding view.
Then press play to plan.
You can then monitor the progress in the console window, which will finally report the log directory,
which you will need to visualize a solution.

## View a Solution
After planning has been completed, the resulting solutions can be viewed.
Browse to the results / log folder of the plan execution and selected a session from the numbered folders.
Note, that the log files will record the outputs of all solutions, so that you can also visualize invalid solutions.

``
$> cd /tmp20200504_214321+0200-templ/

$> ls
0  1  search-statistics.log  solution_analysis.log  specs

$> cd 0

$> ls
20200504-21:43:21+0200-transport-network.status
multicommodity-min-cost-flow-final-flow.gexf
multicommodity-min-cost-flow.problem
transhipment-flow-network.gexf
final_plan.gexf
multicommodity-min-cost-flow.gexf
multicommodity-min-cost-flow.solution
transport-network-solution-0.dot
final_solution_network.gexf
multicommodity-min-cost-flow-init.gexf
templ-mission-relations.dot
transport-network-solution-0.gexf
```

The actual solution is contained in 'final_solution_network.gexf'
