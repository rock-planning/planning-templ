# TemPl: A temporal mission planner for reconfigurable multi-robot systems

TemPl is a planning system for reconfigurable multi-robot systems which is
described in the PhD Thesis "Autonomous Operation of a Reconfigurable Multi-Robot System for Planetary Space
Missions" ([Roehr 2019](http://nbn-resolving.de/urn:nbn:de:gbv:46-00107698-18))
To solve the highly combinatorial optimization problem it combines the
use of knowledge-based engineering, constraint-based programming and linear
programming.

The focus of TemPl is to allow planning and optimization of (currently limited
to medium-sized) missions of reconfigurable robotic agents.

TemPl comes with multiple utility programs such as a graphical interface to
inspect solutions, a mission editor to design solutions (for those not wanting
to deal with the XML format),
and a mission generator to automatically generate simple as well as complex test missions.

## Installation
Please use autoproj to bootstrap and include the package-set
[rock](https://github.com/rock-planning/planning-templ) in your manifest.
Afterwards you can install TemPl via:
```
    amake planning/templ
```

This software remains under development, so if you plan to use it and
experience difficulties, you are encouraged to contact the author of this
library by opening an issue on the repository.

## Testing
For testing the library Boost testing is used, along with the
definition of a number of so-called test-scenarios, i.e. sample missions.

```
$> autoproj test enable planning/templ
$> amake planning/templ
$> cd planning/templ
$> ./build/test/templ-test --log_level=all
```

### Test Scenarios

The folder test/data/scenarios contains a set of missions that cover/validate
different aspects of the mission planner.

## Usage Examples:
After building the library (and if following the instructions given in this document),
the executables are typically available in the corresponding build/ folder and in PATH,
so that you can easily access them via the 'templ-' prefix.

All executables support the '--help' argument to describe their current usage.

### Planner
The actual planner: templ-transport_network_planner

```
allowed options:
  --help                describe arguments
  --mission arg         Path to the mission specification
  --configuration arg   Path to the search configuration file
  --om arg              IRI of the organization model (optional)
  --min_solutions arg   Minimum number of solutions (optional)
```

The default configuration for templ-transport_network_planner can be found under:
test/data/configuration/default-configuration.xml, further
details on configuration options can be found [here](doc/configuration.md).

Every run of the planner will result in the creation of a log folder under /tmp: for the
overall mission a 'spec' subfolder will be created and successful epochs (also
referred to as sessions) will be logged - identified by a counter.
Each epoch folder contains solutions and results to intermediate planning steps.
So check /tmp/<current-time-date>_temp/ for the corresponding folder of the
session that you would like to inspect.

```
$>./build/src/templ-transport_network_planner --mission test/data/scenarios/should_succeed/0.xml --min_solution 1
...

Session 0: remaining flaws: 0
Solution found:
    # session id 0
    # flaws: 0
    # cost: 0

Saving stats in: /tmp/20201111_102602+0100-templ/search-statistics.log
Solution Search (epoch: 1)
    was stopped (e.g. timeout):  no
    found # solutions: 1
    minimum # requested: 1
TemPl:
    # of solutions found: 1
    Check log directory: /tmp/20201111_102602+0100-templ/

```

### Templ GUI
The graphical user interface templ-gui allows you to load solutions, so that you can
inspect them and achieve a better understanding of the solutions.
The graphical interface will also permit the creation of missions via the
MissionEditor.

Details can be found [here](doc/gui.md).

### Solution analysis

Solution analysis allows to recompute the cost analysis that is performed for
each solution.
The command line interface can compute and output the text format of the solution:
```
$>./build/src/templ-solution_analysis --mission test/data/scenarios/should_succeed/0.xml --solution /tmp/20201111_102602+0100-templ/0/final_solution_network.gexf --report
Processing: /tmp/20201111_102602+0100-templ/0/final_solution_network.gexf

Report:
SolutionAnalysis:
    Resulting plan:
        plan:
        - role: AtomicAgent: Payload_0 (Payload)
            Tuple:
                a:
                    lander
                b:
                    t0
                roles (assigned):
                    Payload:
                        0
                    Sherpa:
                        0
                roles (required):
                    Payload:
                        0
                    Sherpa:
                        0

            Tuple:
                a:
                    lander
                b:
                    t1
                roles (assigned):
                    Payload:
                        0
                    Sherpa:
                        0
                roles (required):
                    Payload:
                        0
                    Sherpa:
                        0

            Tuple:
                a:
                    base1
                b:
                    t2
                roles (assigned):
                    Payload:
                        0
                    Sherpa:
                        0
                roles (required):
                    Payload:
                        0

            Tuple:
                a:
                    base1
                b:
                    t3
                roles (assigned):
                    Payload:
                        0
                roles (required):
                    Payload:
                        0

            Tuple:
                a:
                    base1
                b:
                    t4
                roles (assigned):
                    Payload:
                        0

            Tuple:
                a:
                    base1
                b:
                    t5
                roles (assigned):
                    Payload:
                        0

        - role: AtomicAgent: Sherpa_0 (Sherpa)
            Tuple:
                a:
                    lander
                b:
                    t0
                roles (assigned):
                    Payload:
                        0
                    Sherpa:
                        0
                roles (required):
                    Payload:
                        0
                    Sherpa:
                        0

            Tuple:
                a:
                    lander
                b:
                    t1
                roles (assigned):
                    Payload:
                        0
                    Sherpa:
                        0
                roles (required):
                    Payload:
                        0
                    Sherpa:
                        0

            Tuple:
                a:
                    base1
                b:
                    t2
                roles (assigned):
                    Payload:
                        0
                    Sherpa:
                        0
                roles (required):
                    Payload:
                        0

            Tuple:
                a:
                    lander
                b:
                    t3
                roles (assigned):
                    Sherpa:
                        0

            Tuple:
                a:
                    base2
                b:
                    t4
                roles (assigned):
                    Sherpa:
                        0
                roles (required):
                    Sherpa:
                        0

            Tuple:
                a:
                    base2
                b:
                    t5
                roles (assigned):
                    Sherpa:
                        0
                roles (required):
                    Sherpa:
                        0

        Timepoints:
            t0
            t1
            t2
            t3
            t4
            t5
        time horizon: 1.89795e+06
            t0: 0
            t1: 0
            t2: 96452.1
            t3: 194464
            t4: 1.89795e+06
            t5: 1.89795e+06
        cost: 77.9083
            efficacy: 1
            efficiency: 76.4451
            reconfiguration: 1560
            safety: 0.463291
            travel distance: 948193
        # of agents: 2
        # of mobile agents: 1
            ModelPool:
                http://www.rock-robotics.org/2014/01/om-schema#Payload : 1
                http://www.rock-robotics.org/2014/01/om-schema#Sherpa : 1

```

# Further Details
 * [Configuration](doc/configuration.md)
 * [GUI](doc/gui.md)
 * [IO](doc/io.md)

# Contributing, Merge Requests and Issue Tracking

If you wish to report bugs, please use the [issue tracker](https://github.com/rock-planning/planning-templ/issues) at Github.
If you would like to contribute to TemPl, open an issue or a pull request.
Please assign the current maintainer to the reviewer list of the issue or pull
request to ensure notification.
The target branch for pull requests is the 'main' branch. 

Releases will be made available as tags of the main branch.

The recommended workflow to add a new feature, add documentation, or fix a bug is the following:

  * push your changes to a branch (e.g. feature/x, doc/y, or fix/z) of your fork this repository
  * open a pull request to main branch

All new features need to be accompanied by a corresponding unit test.

# Citation

If you use TemPl for a scientific publication, please cite the following paper:

```bibtex
@article{Roehr:2018:IBERAMIA,
author = {Roehr, Thomas M.},
doi = {10.4114/intartif.vol21iss62pp25-39},
journal = {Inteligencia Artificial, Revista Iberoamericana de Inteligencia Artificial},
mendeley-groups = {all},
number = {62},
pages = {25--39},
title = {{A Constraint-based Mission Planning Approach for Reconfigurable Multi-Robot Systems}},
volume = {21},
year = {2018}
}
```

# Copyright

Copyright (c) 2015-2021 Thomas M. Roehr, DFKI GmbH Robotics Innovation Center
