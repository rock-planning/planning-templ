# TemPl: A temporal mission planner for reconfigurable multi-robot systems

TemPl is a planning system for reconfigurable multi-robot systems.
To solve the highly combinatorial optimization problem it combines the 
use of knowledge-based engineering, constraint-based programming and linear
programming.

The current focus of TemPl is to allow planning and optimization of medium sized mission with a
set of so-called atomic agents.

## Installation
Please use autoproj to bootstrap and include the package-set rock.dfki and
dfki.planning in you manifest.
Afterwards you can install templ via:
```
    amake planning/templ
```

This software is still under intensive development. So if you plan to use it,
you are encouraged to contact the author of this library.


## Testing
For testing the library boost testing is being heavily used, along with the
definition of a number of so-called test-scenario, i.e. sample missions.

### Test Scenarios

The folder test/data/scenarios contains a set of missions that try to test
different aspects of the mission planner.

## Usage Examples:
After building the library, the executables are typically available in the
corresponding build/ folder.
All executable support the --help command.

### Planner ###
The actual planner:
templ-transport_network_planner

```
allowed options:
  --help                describe arguments
  --mission arg         Path to the mission specification
  --configuration arg   Path to the search configuration file
  --om arg              IRI of the organization model (optional)
  --min_solutions arg   Minimum number of solutions (optional)
```

The default configuration can be found under: test/data/configuration/default-configuration.xml
Solution analysis allows to recompute the cost analysis that is performed for
each solution: 

```
./build/src/templ-transport_network_planner --mission ./test/data/scenarios/should_succeed/0_deviation-0.xml --min_solutions 3
```

All solutions will result in the creation of log folders under /tmp. For the
overall mission a spec folder will be created and succesful epochs (sessions) will also be logged.
So check /tmp/<current-time-date>_temp/ for the corresponding folder.

### Templ GUI ###
The graphical interface templ-gui allows to read solutions in order to generate
an easier understanding of the solutions. 
Start the GUI and import the *.gexf files, e.g., use final_plan.gexf to import
the final solutions found by the planner.

### Solution analysis ###
Solution analysis allows to recompute the cost analysis that is performed for
each solution:
```
./build/src/templ-solution_analysis test/data/solution_analysis/0_deviation-0.xml /tmp/.../final_plan.gexf
```

# Copyright

Copyright (c) 2015-2018 Thomas M. Roehr, DFKI GmbH Robotics Innovation Center
